import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from scipy.signal import butter, lfilter_zi, lfilter, medfilt
import numpy as np
import json
import psutil
import time
from collections import deque

class TractorFilterNode(Node):
    def __init__(self):
        super().__init__('tractor_filter_node')

        # --- Parametry filtru prędkości ---
        self.declare_parameter('speed_filter_cutoff_hz', 0.8)
        self.declare_parameter('filter_order', 2)
        self.declare_parameter('sampling_frequency_hz', 20.0)
        
        # --- Parametry filtru pozycji ---
        self.declare_parameter('position_filter_cutoff_hz', 0.5)
        
        # --- Parametry filtru headingu ---
        self.declare_parameter('heading_filter_cutoff_hz', 0.3)
        
        # --- Parametry filtra medianowego ---
        self.declare_parameter('median_filter_window', 5)
        self.declare_parameter('median_filter_on_off', False)  # Parametr włączania/wyłączania filtra medianowego
        
        # --- Parametry filtra wykrywania szpilek ---
        self.declare_parameter('spike_filter_enabled', False)  # Włączanie/wyłączanie filtra szpilek
        self.declare_parameter('spike_threshold_speed', 2.0)  # Próg wykrywania szpilek w prędkości (m/s)
        self.declare_parameter('spike_threshold_heading', 10.0)  # Próg wykrywania szpilek w headingu (stopnie)
        self.declare_parameter('spike_min_speed', 0.0)  # Minimalna fizycznie możliwa prędkość (m/s)
        self.declare_parameter('spike_max_speed', 7.0)  # Maksymalna fizycznie możliwa prędkość (m/s)

        # Parametry filtru prędkości
        speed_fpass = self.get_parameter('speed_filter_cutoff_hz').get_parameter_value().double_value
        order = self.get_parameter('filter_order').get_parameter_value().integer_value
        fs = self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value
        
        # Parametry filtru pozycji
        position_fpass = self.get_parameter('position_filter_cutoff_hz').get_parameter_value().double_value
        
        # Parametry filtru headingu
        heading_fpass = self.get_parameter('heading_filter_cutoff_hz').get_parameter_value().double_value
        
        # Parametry filtra medianowego
        self.median_window = self.get_parameter('median_filter_window').get_parameter_value().integer_value
        self.median_filter_enabled = self.get_parameter('median_filter_on_off').get_parameter_value().bool_value
        
        # Parametry filtra wykrywania szpilek
        self.spike_filter_enabled = self.get_parameter('spike_filter_enabled').get_parameter_value().bool_value
        self.spike_threshold_speed = self.get_parameter('spike_threshold_speed').get_parameter_value().double_value
        self.spike_threshold_heading = self.get_parameter('spike_threshold_heading').get_parameter_value().double_value
        self.spike_min_speed = self.get_parameter('spike_min_speed').get_parameter_value().double_value
        self.spike_max_speed = self.get_parameter('spike_max_speed').get_parameter_value().double_value

        # QoS dla danych GPS
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subskrypcja surowych danych GPS ciągnika
        self.subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/tractor',
            self.listener_callback,
            qos_profile=sensor_qos_profile)

        # Publikacja przefiltrowanych danych GPS ciągnika
        self.publisher_ = self.create_publisher(
            GpsRtk, 
            '/gps_rtk_data/tractor_filtered', 
            qos_profile=sensor_qos_profile
        )

        # === Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/tractor_filter_node', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        # === Serwis do włączania/wyłączania filtra szpilek ===
        from std_srvs.srv import SetBool
        self.spike_filter_service = self.create_service(
            SetBool,
            '/mss/filters/tractor/spike_filter_toggle',
            self.toggle_spike_filter_callback
        )

        # Inicjalizacja filtru Butterworth dla prędkości
        self.speed_b, self.speed_a = butter(order, speed_fpass, btype='low', analog=False, fs=fs)
        self.speed_zi = lfilter_zi(self.speed_b, self.speed_a)
        
        # Inicjalizacja filtrów Butterworth dla pozycji (lat/lon)
        self.position_b, self.position_a = butter(order, position_fpass, btype='low', analog=False, fs=fs)
        self.lat_zi = lfilter_zi(self.position_b, self.position_a)
        self.lon_zi = lfilter_zi(self.position_b, self.position_a)
        
        # Inicjalizacja filtru Butterworth dla headingu
        self.heading_b, self.heading_a = butter(order, heading_fpass, btype='low', analog=False, fs=fs)
        self.heading_zi = lfilter_zi(self.heading_b, self.heading_a)
        
        # Inicjalizacja buforów dla filtra medianowego
        self.speed_buffer = deque(maxlen=self.median_window)
        self.lat_buffer = deque(maxlen=self.median_window)
        self.lon_buffer = deque(maxlen=self.median_window)
        self.heading_buffer = deque(maxlen=self.median_window)
        
        # Bufor dla filtra wykrywania szpilek (ostatnie wartości)
        self.last_speed = None
        self.last_heading = None
        self.last_valid_speed = None
        self.last_valid_heading = None

        # Statystyki
        self.filtered_count = 0
        self.spikes_detected_count = 0
        self.last_filter_time = time.time()

        self.get_logger().info(f"Filtr ciągnika uruchomiony.")
        self.get_logger().info(f"Parametry filtru prędkości: cutoff={speed_fpass}Hz, order={order}, fs={fs}Hz")
        self.get_logger().info(f"Parametry filtru pozycji: cutoff={position_fpass}Hz, order={order}, fs={fs}Hz")
        self.get_logger().info(f"Parametry filtru headingu: cutoff={heading_fpass}Hz, order={order}, fs={fs}Hz")
        self.get_logger().info(f"Parametry filtra medianowego: window={self.median_window}, enabled={self.median_filter_enabled}")
        self.get_logger().info(f"Parametry filtra szpilek: enabled={self.spike_filter_enabled}, speed_threshold={self.spike_threshold_speed}m/s, heading_threshold={self.spike_threshold_heading}°")

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status filtrów
            filter_status = "OK"
            if not hasattr(self, 'speed_b') or not hasattr(self, 'speed_a') or not hasattr(self, 'position_b') or not hasattr(self, 'position_a') or not hasattr(self, 'heading_b') or not hasattr(self, 'heading_a'):
                filter_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if filter_status == "ERROR":
                errors.append("Filtry nie zainicjalizowane")
            
            # Sprawdź czy dane są odbierane
            time_since_last_filter = time.time() - self.last_filter_time
            if time_since_last_filter > 10.0:  # Brak danych przez 10s
                warnings.append(f"Brak danych GPS ciągnika przez {time_since_last_filter:.1f}s")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'filter_status': filter_status,
                'speed_filter_cutoff_hz': self.get_parameter('speed_filter_cutoff_hz').get_parameter_value().double_value,
                'position_filter_cutoff_hz': self.get_parameter('position_filter_cutoff_hz').get_parameter_value().double_value,
                'heading_filter_cutoff_hz': self.get_parameter('heading_filter_cutoff_hz').get_parameter_value().double_value,
                'filter_order': self.get_parameter('filter_order').get_parameter_value().integer_value,
                'sampling_frequency_hz': self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value,
                'median_filter_window': self.get_parameter('median_filter_window').get_parameter_value().integer_value,
                'median_filter_enabled': self.get_parameter('median_filter_on_off').get_parameter_value().bool_value,
                'spike_filter_enabled': self.spike_filter_enabled,
                'spike_threshold_speed': self.spike_threshold_speed,
                'spike_threshold_heading': self.spike_threshold_heading,
                'spike_min_speed': self.spike_min_speed,
                'spike_max_speed': self.spike_max_speed,
                'filtered_messages_count': self.filtered_count,
                'spikes_detected_count': self.spikes_detected_count,
                'time_since_last_data': time_since_last_filter,
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

    def normalize_heading(self, heading_deg):
        """Normalizuje heading do zakresu 0-360 stopni."""
        while heading_deg < 0:
            heading_deg += 360
        while heading_deg >= 360:
            heading_deg -= 360
        return heading_deg
    
    def toggle_spike_filter_callback(self, request, response):
        """Callback serwisu do włączania/wyłączania filtra szpilek."""
        try:
            self.spike_filter_enabled = request.data
            response.success = True
            response.message = f"Filtr szpilek {'włączony' if request.data else 'wyłączony'}"
            self.get_logger().info(f"Filtr szpilek {'włączony' if request.data else 'wyłączony'} przez serwis")
        except Exception as e:
            response.success = False
            response.message = f"Błąd podczas przełączania filtra szpilek: {e}"
            self.get_logger().error(response.message)
        return response
    
    def detect_spike_speed(self, current_speed):
        """Wykrywa szpilki w prędkości."""
        if not self.spike_filter_enabled or self.last_speed is None:
            return False, current_speed
            
        # Sprawdź czy prędkość jest poza fizycznie możliwym zakresem
        if current_speed < self.spike_min_speed or current_speed > self.spike_max_speed:
            self.get_logger().warn(f"Wykryto szpilkę prędkości: {current_speed:.2f} m/s (poza zakresem {self.spike_min_speed}-{self.spike_max_speed})")
            return True, self.last_valid_speed if self.last_valid_speed is not None else self.last_speed
        
        # Sprawdź czy zmiana prędkości jest zbyt gwałtowna
        speed_change = abs(current_speed - self.last_speed)
        if speed_change > self.spike_threshold_speed:
            self.get_logger().warn(f"Wykryto szpilkę prędkości: zmiana {speed_change:.2f} m/s (próg: {self.spike_threshold_speed})")
            return True, self.last_valid_speed if self.last_valid_speed is not None else self.last_speed
            
        return False, current_speed
    
    def detect_spike_heading(self, current_heading):
        """Wykrywa szpilki w headingu."""
        if not self.spike_filter_enabled or self.last_heading is None:
            return False, current_heading
            
        # Normalizuj headingu do porównania
        current_heading_norm = self.normalize_heading(current_heading)
        last_heading_norm = self.normalize_heading(self.last_heading)
        
        # Oblicz różnicę uwzględniając przejście przez 0/360
        heading_diff = abs(current_heading_norm - last_heading_norm)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
            
        if heading_diff > self.spike_threshold_heading:
            self.get_logger().warn(f"Wykryto szpilkę headingu: zmiana {heading_diff:.1f}° (próg: {self.spike_threshold_heading}°)")
            return True, self.last_valid_heading if self.last_valid_heading is not None else self.last_heading
            
        return False, current_heading

    def listener_callback(self, msg):
        """Callback dla surowych danych GPS ciągnika."""
        self.get_logger().debug(f'>>> Otrzymano dane GPS ciągnika! Prędkość surowa: {msg.speed_mps:.2f} m/s, Lat: {msg.latitude_deg:.8f}, Lon: {msg.longitude_deg:.8f}, Heading: {msg.heading_deg:.2f}°')
        
        # === FILTR WYKRYWANIA SZPILEK ===
        # Wykryj szpilki w prędkości i headingu
        speed_spike_detected, corrected_speed = self.detect_spike_speed(msg.speed_mps)
        heading_spike_detected, corrected_heading = self.detect_spike_heading(msg.heading_deg)
        
        # Użyj skorygowanych wartości jeśli wykryto szpilki
        input_speed = corrected_speed if speed_spike_detected else msg.speed_mps
        input_heading = corrected_heading if heading_spike_detected else msg.heading_deg
        
        # Aktualizuj statystyki szpilek
        if speed_spike_detected or heading_spike_detected:
            self.spikes_detected_count += 1
        
        # Sprawdź czy filtr medianowy jest włączony
        if self.median_filter_enabled:
            # Dodaj skorygowane dane (po filtrze szpilek) do buforów filtra medianowego
            self.speed_buffer.append(input_speed)
            self.lat_buffer.append(msg.latitude_deg)
            self.lon_buffer.append(msg.longitude_deg)
            self.heading_buffer.append(self.normalize_heading(input_heading))
            
            # Zastosuj filtr medianowy
            median_speed = np.median(list(self.speed_buffer))
            median_lat = np.median(list(self.lat_buffer))
            median_lon = np.median(list(self.lon_buffer))
            median_heading = np.median(list(self.heading_buffer))
            
            # Filtruj prędkość (najpierw medianowy, potem Butterworth)
            filtered_speed, self.speed_zi = lfilter(self.speed_b, self.speed_a, [median_speed], zi=self.speed_zi)
            
            # Filtruj pozycję (najpierw medianowy, potem Butterworth)
            filtered_lat, self.lat_zi = lfilter(self.position_b, self.position_a, [median_lat], zi=self.lat_zi)
            filtered_lon, self.lon_zi = lfilter(self.position_b, self.position_a, [median_lon], zi=self.lon_zi)
            
            # Filtruj heading (najpierw medianowy, potem Butterworth)
            filtered_heading, self.heading_zi = lfilter(self.heading_b, self.heading_a, [median_heading], zi=self.heading_zi)
        else:
            # Filtr medianowy wyłączony - używaj skorygowanych danych (po filtrze szpilek)
            # Filtruj prędkość (tylko Butterworth)
            filtered_speed, self.speed_zi = lfilter(self.speed_b, self.speed_a, [input_speed], zi=self.speed_zi)
            
            # Filtruj pozycję (tylko Butterworth)
            filtered_lat, self.lat_zi = lfilter(self.position_b, self.position_a, [msg.latitude_deg], zi=self.lat_zi)
            filtered_lon, self.lon_zi = lfilter(self.position_b, self.position_a, [msg.longitude_deg], zi=self.lon_zi)
            
            # Filtruj heading (tylko Butterworth)
            filtered_heading, self.heading_zi = lfilter(self.heading_b, self.heading_a, [self.normalize_heading(input_heading)], zi=self.heading_zi)

        # Stwórz nową wiadomość z przefiltrowanymi danymi
        filtered_msg = GpsRtk()
        filtered_msg.header = msg.header
        filtered_msg.gps_time = msg.gps_time
        filtered_msg.rtk_status = msg.rtk_status
        filtered_msg.latitude_deg = filtered_lat[0]  # Filtrowana pozycja
        filtered_msg.longitude_deg = filtered_lon[0]  # Filtrowana pozycja
        filtered_msg.altitude_m = msg.altitude_m
        filtered_msg.heading_deg = self.normalize_heading(filtered_heading[0])  # Filtrowany heading
        filtered_msg.speed_mps = filtered_speed[0]  # Filtrowana prędkość

        self.publisher_.publish(filtered_msg)
        
        # Aktualizuj statystyki
        self.filtered_count += 1
        self.last_filter_time = time.time()
        
        # Aktualizuj bufory dla filtra szpilek
        self.last_speed = msg.speed_mps
        self.last_heading = msg.heading_deg
        if not speed_spike_detected:
            self.last_valid_speed = msg.speed_mps
        if not heading_spike_detected:
            self.last_valid_heading = msg.heading_deg
        
        # Logowanie z informacją o stanie filtrów
        spike_info = ""
        if speed_spike_detected or heading_spike_detected:
            spike_info = f" [SZPILKI: speed={speed_spike_detected}, heading={heading_spike_detected}]"
        
        if self.median_filter_enabled:
            self.get_logger().debug(f'Przefiltrowane dane ciągnika: Speed={filtered_speed[0]:.2f} m/s (median: {median_speed:.2f}), Lat={filtered_lat[0]:.8f}, Lon={filtered_lon[0]:.8f}, Heading={self.normalize_heading(filtered_heading[0]):.2f}° (median: {median_heading:.2f}°){spike_info}')
        else:
            self.get_logger().debug(f'Przefiltrowane dane ciągnika: Speed={filtered_speed[0]:.2f} m/s (raw: {msg.speed_mps:.2f}, median OFF), Lat={filtered_lat[0]:.8f}, Lon={filtered_lon[0]:.8f}, Heading={self.normalize_heading(filtered_heading[0]):.2f}° (raw: {msg.heading_deg:.2f}°, median OFF){spike_info}')

def main(args=None):
    rclpy.init(args=args)
    node = TractorFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()