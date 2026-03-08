#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import socket
import base64
import serial
import threading
import time
from datetime import datetime, timezone

from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import String

class GpsRtkNode(Node):
    def __init__(self):
        super().__init__('gps_rtk_node')

 
        self.declare_parameter('serial_port', '/dev/ttyAMA0')        
        self.declare_parameter('baud_rate', 460800)
        self.declare_parameter('ntrip_ip', 'system.asgeupos.pl')
        self.declare_parameter('ntrip_port', 8086)
        self.declare_parameter('ntrip_mountpoint', 'RTK4G_MULTI_RTCM32') 
        self.declare_parameter('ntrip_user', 'pweiti/turbobolek')
        self.declare_parameter('ntrip_password', 'Globus7142001')
        self.declare_parameter('gngga_ntrip_interval', 10.0)
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('heading_dual', 1)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.ntrip_ip = self.get_parameter('ntrip_ip').get_parameter_value().string_value
        self.ntrip_port = self.get_parameter('ntrip_port').get_parameter_value().integer_value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').get_parameter_value().string_value
        self.ntrip_user = self.get_parameter('ntrip_user').get_parameter_value().string_value
        self.ntrip_password = self.get_parameter('ntrip_password').get_parameter_value().string_value
        self.gngga_ntrip_interval = self.get_parameter('gngga_ntrip_interval').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.heading_dual = self.get_parameter('heading_dual').get_parameter_value().integer_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(GpsRtk, '/gps_rtk_data/tractor', qos_profile)

        self.health_pub = self.create_publisher(String, '/mss/node_health/gps_rtk_node', qos_profile)
        self.health_timer = self.create_timer(5.0, self.publish_health)

        # struktura do przechowywania danych
        self.latest_gps_data = {
            'gps_utc_time_str': None,
            'rtk_status': 0,
            'latitude_deg': 0.0,
            'longitude_deg': 0.0,
            'altitude_m': 0.0,
            'speed_mps': 0.0,
            'heading_deg': 0.0,
            'timestamp_gngga': 0.0,
            'timestamp_agric': 0.0, 
            'timestamp_gnvtg': 0.0 
        }
        self.data_lock = threading.Lock()

        self.rtk_serial = None
        self.ntrip_socket = None
        self.gngga_for_ntrip_initial = None
        self.last_gngga_to_ntrip_str = None
        self.last_gngga_send_time = 0.0
        self.stop_threads_event = threading.Event()

        if self._initialize_serial() and self._initialize_ntrip():
            self.thread_rtk_reader = threading.Thread(target=self._rtk_reader_thread_func, daemon=True)
            self.thread_ntrip_reader = threading.Thread(target=self._ntrip_reader_thread_func, daemon=True)
            self.thread_rtk_reader.start()
            self.thread_ntrip_reader.start()
            self.get_logger().info("Wątki RTK i NTRIP uruchomione.")
        else:
            self.get_logger().error("Inicjalizacja nie powiodła się. Node nie będzie działać poprawnie.")

        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self._publish_gps_data_callback)
        self.get_logger().info(f"GPS RTK Node uruchomiony. Publikowanie na '/gps_rtk_data/tractor' z częstotliwością {self.publish_frequency} Hz.")
        self.get_logger().info("Node oczekuje teraz na wiadomości GNGGA, GNVTG oraz AGRIC.")

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            import json
            import psutil
            
            # Sprawdź status portu szeregowego
            serial_status = "OK" if (self.rtk_serial and self.rtk_serial.is_open) else "ERROR"
            
            # Sprawdź status połączenia NTRIP
            ntrip_status = "OK" if self.ntrip_socket else "ERROR"
            
            # Sprawdź status wątków
            threads_status = "OK"
            if hasattr(self, 'thread_rtk_reader') and hasattr(self, 'thread_ntrip_reader'):
                if not self.thread_rtk_reader.is_alive() or not self.thread_ntrip_reader.is_alive():
                    threads_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if serial_status == "ERROR":
                errors.append("Port szeregowy nieaktywny")
            if ntrip_status == "ERROR":
                warnings.append("Brak połączenia NTRIP")
            if threads_status == "ERROR":
                errors.append("Wątki nieaktywne")
            
            # Sprawdź świeżość danych GPS
            current_time = self.get_clock().now().nanoseconds / 1e9
            if hasattr(self, 'latest_gps_data') and 'timestamp_gngga' in self.latest_gps_data:
                time_since_gngga = current_time - self.latest_gps_data['timestamp_gngga']
                if time_since_gngga > 10.0:  # Więcej niż 10 sekund
                    warnings.append(f"Dane GPS nieaktualne ({time_since_gngga:.1f}s)")
            
            # Sprawdź świeżość danych GNVTG
            if hasattr(self, 'latest_gps_data') and 'timestamp_gnvtg' in self.latest_gps_data:
                time_since_gnvtg = current_time - self.latest_gps_data['timestamp_gnvtg']
                if time_since_gnvtg > 10.0:  # Więcej niż 10 sekund
                    warnings.append(f"Dane prędkości GNVTG nieaktualne ({time_since_gnvtg:.1f}s)")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': current_time,
                'serial_status': serial_status,
                'ntrip_status': ntrip_status,
                'threads_status': threads_status,
                'gps_data_age': time_since_gngga if 'time_since_gngga' in locals() else 0.0,
                'gnvtg_data_age': time_since_gnvtg if 'time_since_gnvtg' in locals() else 0.0,
                'errors': errors,
                'warnings': warnings,
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent
            }
            
            # Opublikuj health status
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikowania health status: {e}")

    def _rtk_reader_thread_func(self):
        """
        Główny wątek odczytujący dane z portu szeregowego.
        Parsuje wiadomości GNGGA oraz AGRIC
        """
        self.get_logger().info("Wątek czytnika RTK uruchomiony.")
        while not self.stop_threads_event.is_set() and rclpy.ok():
            try:
                if not self.rtk_serial or not self.rtk_serial.is_open:
                    self.get_logger().warn("Port szeregowy nie jest otwarty. Próba ponownego otwarcia za 5s.")
                    time.sleep(5)
                    if not self._initialize_serial(): continue
                
                line_bytes = self.rtk_serial.readline()
                if line_bytes:
                    decoded_line = line_bytes.decode('ascii', errors='ignore').strip()
                    if not decoded_line: continue

                    current_host_time = self.get_clock().now().nanoseconds / 1e9

                    with self.data_lock:
                        if decoded_line.startswith("$GNGGA"):
                            # Sprawdź czy linia nie jest uszkodzona
                            if decoded_line.count(',') < 9 or '$' in decoded_line[1:]:
                                self.get_logger().warn(f"Pominięto uszkodzoną linię GNGGA: {decoded_line}")
                                continue
                            parts = decoded_line.split(',')
                            if len(parts) > 9:
                                try:
                                    self.latest_gps_data['gps_utc_time_str'] = parts[1]
                                    self.latest_gps_data['latitude_deg'] = self._nmea_to_decimal_degrees(parts[2], parts[3])
                                    self.latest_gps_data['longitude_deg'] = self._nmea_to_decimal_degrees(parts[4], parts[5])
                                    self.latest_gps_data['rtk_status'] = int(parts[6]) if parts[6] else 0
                                    self.latest_gps_data['altitude_m'] = float(parts[9]) if parts[9] else 0.0
                                    self.latest_gps_data['timestamp_gngga'] = current_host_time
                                    self.last_gngga_to_ntrip_str = decoded_line + "\r\n"
                                except (ValueError, IndexError) as e:
                                    self.get_logger().warn(f"Błąd parsowania GNGGA '{decoded_line}': {e}")
                        
                        # --- PARSOWANIE WIADOMOŚCI GNVTG (prędkość) ---
                        elif decoded_line.startswith("$GNVTG"):
                            try:
                                # Sprawdź czy linia nie jest uszkodzona
                                if decoded_line.count(',') < 8:
                                    self.get_logger().warn(f"Pominięto uszkodzoną linię GNVTG: {decoded_line}")
                                    continue
                                
                                parts = decoded_line.split(',')                                
                                # GNVTG format: $GNVTG,track_deg,T,mag_deg,M,speed_knots,N,speed_kmh,K,mode*checksum
                                # Indeksy: 0=header, 1=track, 2=T, 3=mag, 4=M, 5=speed_knots, 6=N, 7=speed_kmh, 8=K, 9=mode
                                
                                if len(parts) >= 8:
                                    # Sprawdź różne możliwe pozycje prędkości w km/h
                                    speed_kmh = None
                                    
                                    # Próba 1: Pole 7 (standardowe)
                                    if len(parts) > 7 and parts[7] and parts[7] != 'K':
                                        speed_str = parts[7].strip()
                                        if self._validate_numeric_string(speed_str, "prędkość"):
                                            speed_kmh = float(speed_str)
                                    
                                    # Próba 2: Pole 6 może zawierać prędkość (gdy brak przecinka)
                                    if speed_kmh is None and len(parts) > 6 and parts[6]:
                                        speed_str = parts[6].strip()
                                        # Sprawdź czy zawiera 'N' i liczbę (np. "N0.01150")
                                        if 'N' in speed_str and len(speed_str) > 1:
                                            # Wyciągnij liczbę po 'N'
                                            try:
                                                numeric_part = speed_str.split('N')[1]
                                                if self._validate_numeric_string(numeric_part, "prędkość"):
                                                    speed_kmh = float(numeric_part)
                                            except (ValueError, IndexError):
                                                pass
                                    
                                    # Próba 3: Pole 8 może zawierać prędkość (gdy struktura jest przesunięta)
                                    if speed_kmh is None and len(parts) > 8 and parts[8] and parts[8] != 'K':
                                        speed_str = parts[8].strip()
                                        if self._validate_numeric_string(speed_str, "prędkość"):
                                            speed_kmh = float(speed_str)
                                    
                                    if speed_kmh is not None:
                                        # Konwersja z km/h na m/s
                                        self.latest_gps_data['speed_mps'] = speed_kmh / 3.6
                                        self.latest_gps_data['timestamp_gnvtg'] = current_host_time
                                        self.get_logger().debug(f"GNVTG: Prędkość {speed_kmh} km/h -> {self.latest_gps_data['speed_mps']:.2f} m/s")
                                    else:
                                        self.get_logger().warn(f"GNVTG: Nie znaleziono prawidłowej prędkości w wiadomości: {decoded_line}", throttle_duration_sec=5)
                                        self.get_logger().debug(f"GNVTG: Pola: {parts}")
                            except (ValueError, IndexError) as e:
                                self.get_logger().warn(f"Błąd parsowania GNVTG: {e} w wiadomości: {decoded_line}")
                        
                        # --- PARSOWANIE WIADOMOŚCI AGRIC (tylko kurs) ---
                        elif decoded_line.startswith("#AGRICA"):
                            try:
                                data_part = decoded_line.split(';')[1].split('*')[0]
                                agric_parts = data_part.split(',')
                                
                                if self.heading_dual == 1:
                                    if len(agric_parts) < 21:
                                        self.get_logger().warn(f"AGRIC (Dual): Za mało pól ({len(agric_parts)}), oczekiwano minimum 21", throttle_duration_sec=5)
                                        continue
                                    
                                    try:
                                        heading_status = int(agric_parts[11 - 2]) # Pole 9
                                    except (ValueError, IndexError) as e:
                                        self.get_logger().warn(f"AGRIC (Dual): Błąd parsowania statusu kursu: {e}", throttle_duration_sec=5)
                                        continue
                                    
                                    if heading_status not in [69, 420]: # Tylko jeśli status to FIXED lub FLOAT
                                        try:
                                            heading_str = agric_parts[21 - 2].strip() # Pole 19
                                            
                                            if not self._validate_numeric_string(heading_str, "kurs (dual)"):
                                                self.get_logger().warn(f"AGRIC (Dual): Nieprawidłowy format kursu: '{heading_str}'", throttle_duration_sec=5)
                                                continue
                                            
                                            self.latest_gps_data['heading_deg'] = float(heading_str)
                                            self.latest_gps_data['timestamp_agric'] = current_host_time
                                            self.get_logger().debug(f"AGRIC (Dual): Kurs {self.latest_gps_data['heading_deg']:.2f}°")
                                        except (ValueError, IndexError) as e:
                                            self.get_logger().warn(f"AGRIC (Dual): Błąd konwersji kursu '{agric_parts[21 - 2]}': {e}", throttle_duration_sec=5)
                                            continue
                                    else:
                                        self.get_logger().warn(f"AGRIC (Dual): Status kursu ('{heading_status}') nie jest poprawny. Ignoruję kurs.", throttle_duration_sec=5)
                                
                                else:
                                    if len(agric_parts) < 51:
                                        self.get_logger().warn(f"AGRIC (Single): Za mało pól ({len(agric_parts)}), oczekiwano minimum 51", throttle_duration_sec=5)
                                        continue

                                    # Pobieramy aktualną prędkość (już przetworzoną z GNVTG)
                                    current_speed_mps = self.latest_gps_data.get('speed_mps', 0.0)
                                    
                                    if current_speed_mps > 0.5:
                                        try:
                                            heading_str = agric_parts[51 - 2].strip() # Pole 49
                                            
                                            if not self._validate_numeric_string(heading_str, "kurs (single)"):
                                                self.get_logger().warn(f"AGRIC (Single): Nieprawidłowy format kursu: '{heading_str}'", throttle_duration_sec=5)
                                                continue
                                                
                                            self.latest_gps_data['heading_deg'] = float(heading_str)
                                            self.latest_gps_data['timestamp_agric'] = current_host_time
                                            self.get_logger().debug(f"AGRIC (Single): Kurs {self.latest_gps_data['heading_deg']:.2f}° przy {current_speed_mps:.2f} m/s")
                                            
                                        except (ValueError, IndexError) as e:
                                            self.get_logger().warn(f"AGRIC (Single): Błąd konwersji kursu '{agric_parts[51 - 2]}': {e}", throttle_duration_sec=5)
                                            continue
                                    else:
                                        self.latest_gps_data['timestamp_agric'] = current_host_time
                                        self.get_logger().debug(f"AGRIC (Single): Prędkość {current_speed_mps:.2f} m/s <= 0.5 m/s. Kurs nie jest aktualizowany (używany ostatni znany).", throttle_duration_sec=1)
                                        
                            except (ValueError, IndexError) as e:
                                self.get_logger().warn(f"Błąd parsowania AGRIC: {e} w wiadomości: {decoded_line}", throttle_duration_sec=5)
                        # --- KONIEC PARSOWANIA ---

                    if self.last_gngga_to_ntrip_str and (time.time() - self.last_gngga_send_time > self.gngga_ntrip_interval):
                        if self.ntrip_socket:
                            try:
                                self.get_logger().info(f"NTRIP_UPDATE: Wysyłanie GNGGA: {self.last_gngga_to_ntrip_str.strip()}")
                                self.ntrip_socket.sendall(self.last_gngga_to_ntrip_str.encode('ascii'))
                                self.last_gngga_send_time = time.time()
                            except socket.error as se_ntrip:
                                self.get_logger().error(f"Błąd gniazda podczas wysyłania GNGGA do NTRIP: {se_ntrip}")
                                self.ntrip_socket.close()
                                self.ntrip_socket = None
                                if not self._initialize_ntrip():
                                    self.get_logger().warn("Ponowna inicjalizacja NTRIP nie powiodła się.")
                            except Exception as e:
                                self.get_logger().error(f"Inny błąd wysyłania GNGGA: {e}")
                        else:
                            self.get_logger().warn("Próba wysłania GNGGA, ale gniazdo NTRIP nie jest aktywne. Próba re-inicjalizacji.")
                            if not self._initialize_ntrip():
                                self.get_logger().warn("Ponowna inicjalizacja NTRIP nie powiodła się.")

            except serial.SerialException as e:
                self.get_logger().error(f"Błąd portu szeregowego w wątku RTK: {e}")
                if self.rtk_serial and self.rtk_serial.is_open: self.rtk_serial.close()
                self.rtk_serial = None
                time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"Nieoczekiwany błąd w wątku RTK: {e}", exc_info=True)
                time.sleep(1)
        self.get_logger().info("Wątek czytnika RTK zakończony.")
    
    
    def _publish_gps_data_callback(self):
        msg = GpsRtk()
        current_ros_time = self.get_clock().now()

        with self.data_lock:
            # Sprawdzamy świeżość danych z trzech źródeł
            time_since_gngga = current_ros_time.nanoseconds / 1e9 - self.latest_gps_data['timestamp_gngga']
            time_since_agric = current_ros_time.nanoseconds / 1e9 - self.latest_gps_data['timestamp_agric']
            time_since_gnvtg = current_ros_time.nanoseconds / 1e9 - self.latest_gps_data['timestamp_gnvtg']

            # Publikujemy, tylko jeśli mamy aktualne dane z GNGGA (pozycja), AGRIC (kurs) i GNVTG (prędkość)
            if time_since_gngga < 2.0 and time_since_agric < 2.0 and time_since_gnvtg < 2.0:
                msg.header.stamp = current_ros_time.to_msg()
                msg.header.frame_id = "gps_link"

                msg.gps_time = self._convert_gps_utc_to_ros_time(self.latest_gps_data['gps_utc_time_str'])
                msg.rtk_status = self.latest_gps_data['rtk_status']
                msg.latitude_deg = self.latest_gps_data['latitude_deg']
                msg.longitude_deg = self.latest_gps_data['longitude_deg']
                msg.altitude_m = self.latest_gps_data['altitude_m']
                
                # Używamy danych z wiadomości GNVTG (prędkość) i AGRIC (kurs)
                msg.speed_mps = self.latest_gps_data['speed_mps']
                msg.heading_deg = self.latest_gps_data['heading_deg']

                self.publisher_.publish(msg)
            else:
                 self.get_logger().debug(f"Czekam na świeże dane: GNGGA_age={time_since_gngga:.2f}s, AGRIC_age={time_since_agric:.2f}s, GNVTG_age={time_since_gnvtg:.2f}s")


    def _initialize_serial(self):
        self.get_logger().info(f"Otwieranie portu szeregowego {self.serial_port} z prędkością {self.baud_rate}...")
        try:
            self.rtk_serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1) # Krótki timeout dla readline
            self.get_logger().info(f"Port szeregowy {self.serial_port} otwarty.")

            self.get_logger().info("Oczekiwanie na pierwszą poprawną wiadomość GNGGA z modułu RTK...")
            start_time = time.time()
            initial_gngga_found = False
            while time.time() - start_time < 30:
                if self.stop_threads_event.is_set(): return False
                line_bytes = self.rtk_serial.readline()
                if line_bytes:
                    str_nmea = line_bytes.decode("ascii", errors='ignore').strip()
                    if str_nmea.startswith("$GNGGA"):
                        parts = str_nmea.split(',')
                        if len(parts) > 6 and len(parts[6]) and parts[6] != '0':
                            self.gngga_for_ntrip_initial = str_nmea + "\r\n"
                            self.last_gngga_to_ntrip_str = self.gngga_for_ntrip_initial
                            self.get_logger().info(f"Znaleziono początkowe GNGGA: {str_nmea}")
                            initial_gngga_found = True
                            return True
            if not initial_gngga_found:
                self.get_logger().error("Nie udało się uzyskać początkowej wiadomości GNGGA w ciągu 30s.")
                return False
        except serial.SerialException as e:
            self.get_logger().error(f"Błąd otwarcia lub konfiguracji portu szeregowego {self.serial_port}: {e}")
            return False
        return True

    def _initialize_ntrip(self):
        if not self.gngga_for_ntrip_initial:
            self.get_logger().error("Brak początkowej wiadomości GNGGA do inicjalizacji NTRIP.")
            return False

        self.ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ntrip_socket.settimeout(15.0) 
        try:
            self.get_logger().info(f"Łączenie z serwerem NTRIP {self.ntrip_ip}:{self.ntrip_port}...")
            self.ntrip_socket.connect((self.ntrip_ip, self.ntrip_port))
            self.get_logger().info("Połączono z serwerem NTRIP.")

            user_pwd_encoded = base64.b64encode(bytes(self.ntrip_user + ':' + self.ntrip_password, 'utf-8')).decode("utf-8")
            http_request = (
                f"GET /{self.ntrip_mountpoint} HTTP/1.1\r\n"
                f"User-Agent: NTRIP ROS2Client/1.0\r\n"
                f"Authorization: Basic {user_pwd_encoded}\r\n"
                f"Connection: close\r\n\r\n"
            )
            self.get_logger().info("Wysyłanie żądania autoryzacji NTRIP...")
            self.ntrip_socket.sendall(http_request.encode())
            response = self.ntrip_socket.recv(2048)
            response_str = response.decode('ascii', errors='ignore')
            self.get_logger().info(f"Odpowiedź serwera NTRIP: {response_str.strip()}")

            if "200 OK" not in response_str:
                self.get_logger().error(f"Błąd autoryzacji NTRIP lub serwer nie jest gotowy. Odpowiedź: {response_str.strip()}")
                self.ntrip_socket.close()
                return False
            
            self.get_logger().info("Autoryzacja NTRIP pomyślna.")
            self.get_logger().info(f"Wysyłanie początkowej wiadomości GNGGA do NTRIP: {self.gngga_for_ntrip_initial.strip()}")
            self.ntrip_socket.sendall(self.gngga_for_ntrip_initial.encode())
            self.last_gngga_send_time = time.time()
            return True

        except socket.error as e:
            self.get_logger().error(f"Błąd połączenia z serwerem NTRIP: {e}")
            if self.ntrip_socket: self.ntrip_socket.close()
            return False
        except Exception as e:
            self.get_logger().error(f"Inny błąd podczas konfiguracji NTRIP: {e}")
            if self.ntrip_socket: self.ntrip_socket.close()
            return False

    def _ntrip_reader_thread_func(self):
        self.get_logger().info("Wątek czytnika NTRIP uruchomiony.")
        while not self.stop_threads_event.is_set() and rclpy.ok():
            try:
                if not self.ntrip_socket:
                    self.get_logger().warn("Gniazdo NTRIP nie jest aktywne. Oczekiwanie na re-inicjalizację.")
                    time.sleep(5) # Poczekaj na główny wątek lub wątek RTK, aby spróbować ponownie połączyć
                    if not self.ntrip_socket:
                         if not self._initialize_ntrip():
                              self.get_logger().warn("Ponowna inicjalizacja NTRIP w wątku NTRIP nie powiodła się.")
                              time.sleep(5) 
                              continue
                
                data = self.ntrip_socket.recv(4096)
                if data:
                    if self.rtk_serial and self.rtk_serial.is_open:
                        self.rtk_serial.write(data)
                    else:
                        self.get_logger().warn("Otrzymano dane NTRIP, ale port szeregowy RTK nie jest otwarty.")
                else:
                    self.get_logger().warn("Połączenie NTRIP zamknięte przez serwer (recv zwrócił 0). Próba ponownego połączenia.")
                    self.ntrip_socket.close()
                    self.ntrip_socket = None
                    time.sleep(5) 
            except socket.timeout:
                self.get_logger().debug("Timeout podczas odbierania danych NTRIP. Kontynuuję...")
                continue
            except socket.error as e:
                self.get_logger().error(f"Błąd gniazda w wątku NTRIP: {e}")
                if self.ntrip_socket: self.ntrip_socket.close()
                self.ntrip_socket = None 
                time.sleep(5)
            except serial.SerialException as e:
                 self.get_logger().error(f"Błąd portu szeregowego przy zapisie danych NTRIP: {e}")
                 if self.rtk_serial and self.rtk_serial.is_open: self.rtk_serial.close()
                 self.rtk_serial = None
                 time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"Nieoczekiwany błąd w wątku NTRIP: {e}", exc_info=True)
                time.sleep(1)
        self.get_logger().info("Wątek czytnika NTRIP zakończony.")

    def _validate_numeric_string(self, value_str, field_name="value"):
        """
        Waliduje czy string może być przekonwertowany na float.
        Zwraca True jeśli string jest poprawny, False w przeciwnym razie.
        """
        if not value_str or not isinstance(value_str, str):
            return False
        
        value_str = value_str.strip()
        if not value_str:
            return False
        
        # Sprawdź czy zawiera tylko dozwolone znaki
        allowed_chars = set('0123456789.-+eE')
        if not all(c in allowed_chars for c in value_str):
            return False
        
        # Sprawdź czy nie ma podwójnych kropek lub innych błędów formatu
        if value_str.count('.') > 1:
            return False
        
        # Sprawdź czy nie zaczyna się od kropki (oprócz .0)
        if value_str.startswith('.') and len(value_str) > 1:
            return False
        
        # Sprawdź czy nie kończy się kropką
        if value_str.endswith('.'):
            return False
        
        # Sprawdź czy nie ma podwójnych znaków +/-
        if value_str.count('+') > 1 or value_str.count('-') > 1:
            return False
        
        # Sprawdź czy + i - nie są w środku (oprócz notacji naukowej)
        if '+' in value_str[1:] and 'e' not in value_str.lower():
            return False
        if '-' in value_str[1:] and 'e' not in value_str.lower():
            return False
        
        return True

    def _nmea_to_decimal_degrees(self, nmea_coord, hemisphere):
        if not nmea_coord:
            return 0.0
        try:
            coord = float(nmea_coord)
            degrees = int(coord / 100)
            minutes = coord - (degrees * 100)
            decimal_degrees = degrees + (minutes / 60.0)
            if hemisphere.upper() in ['S', 'W']:
                decimal_degrees *= -1.0
            return decimal_degrees
        except ValueError:
            self.get_logger().warn(f"Nie można przekonwertować współrzędnej NMEA: {nmea_coord}")
            return 0.0

    def _convert_gps_utc_to_ros_time(self, utc_time_str):
        if not utc_time_str or '.' not in utc_time_str:
            return self.get_clock().now().to_msg() 

        try:
            now_utc = datetime.now(timezone.utc)
            
            hour = int(utc_time_str[0:2])
            minute = int(utc_time_str[2:4])
            second = int(utc_time_str[4:6])
            subsecond_str = utc_time_str.split('.')[1]
            nanosecond = int(subsecond_str.ljust(9, '0')[:9])

            gps_datetime = now_utc.replace(hour=hour, minute=minute, second=second, microsecond=nanosecond // 1000, tzinfo=timezone.utc)
            
            ros_time = GpsRtk().header.stamp
            ros_time.sec = int(gps_datetime.timestamp())
            ros_time.nanosec = gps_datetime.microsecond * 1000
            return ros_time
        except ValueError as e:
            self.get_logger().warn(f"Błąd konwersji czasu GPS UTC '{utc_time_str}': {e}")
            return self.get_clock().now().to_msg()

    def destroy_node(self):
        self.get_logger().info("Zamykanie node'a GPS RTK...")
        self.stop_threads_event.set()

        if hasattr(self, 'thread_rtk_reader') and self.thread_rtk_reader.is_alive():
            self.thread_rtk_reader.join(timeout=2.0)
        if hasattr(self, 'thread_ntrip_reader') and self.thread_ntrip_reader.is_alive():
            self.thread_ntrip_reader.join(timeout=2.0)
        
        if self.rtk_serial and self.rtk_serial.is_open:
            self.rtk_serial.close()
            self.get_logger().info("Port szeregowy RTK zamknięty.")
        if self.ntrip_socket:
            try:
                self.ntrip_socket.shutdown(socket.SHUT_RDWR)
            except socket.error: pass
            self.ntrip_socket.close()
            self.get_logger().info("Połączenie NTRIP zamknięte.")
        
        super().destroy_node()
        self.get_logger().info("Node GPS RTK zamknięty.")

def main(args=None):
    rclpy.init(args=args)
    gps_rtk_node = None
    try:
        gps_rtk_node = GpsRtkNode()
        rclpy.spin(gps_rtk_node)
    except KeyboardInterrupt:
        print("Przerwano przez użytkownika (Ctrl+C)")
    except Exception as e:
        if gps_rtk_node:
            gps_rtk_node.get_logger().fatal(f"Krytyczny błąd w main: {e}", exc_info=True)
        else:
            print(f"Krytyczny błąd przed inicjalizacją node'a: {e}")
    finally:
        if gps_rtk_node:
            gps_rtk_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
