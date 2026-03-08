#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from my_robot_interfaces.msg import GpsRtk, DistanceMetrics, Gear
import time
import math

class PositionControllerNode(Node):
    def __init__(self):
        super().__init__('position_controller_node')
        
        # --- Parametry konfiguracyjne ---
        self.declare_parameter('target_distance', 0.0)  # m - docelowa odległość wzdłużna
        self.declare_parameter('position_tolerance', 5.0)  # m - tolerancja pozycji
        self.declare_parameter('speed_tolerance', 1.5)  # m/s - tolerancja prędkości (1 km/h)
        
        # Parametry PID - domyślne (będą nadpisywane przez parametry dla półbiegów)
        self.declare_parameter('Kp', 0.35)  # wzmocnienie proporcjonalne
        self.declare_parameter('Ki', 0.0)  # wzmocnienie całkujące
        self.declare_parameter('Kd', 0.0)  # wzmocnienie różniczkujące
        
        # Parametry PID dla poszczególnych półbiegów
        self.declare_parameter('gear1_Kp', 0.34)  # Półbieg 1 - wolny, stabilny
        self.declare_parameter('gear1_Ki', 0.0)
        self.declare_parameter('gear1_Kd', 0.0)
        
        self.declare_parameter('gear2_Kp', 0.37)  # Półbieg 2 - średni
        self.declare_parameter('gear2_Ki', 0.0)
        self.declare_parameter('gear2_Kd', 0.0)
        
        self.declare_parameter('gear3_Kp', 0.35)  # Półbieg 3 - szybszy
        self.declare_parameter('gear3_Ki', 0.0)
        self.declare_parameter('gear3_Kd', 0.0)
        
        self.declare_parameter('gear4_Kp', 0.36)  # Półbieg 4 - najszybszy
        self.declare_parameter('gear4_Ki', 0.0)
        self.declare_parameter('gear4_Kd', 0.0)
        
        # Parametry max_speed dla każdego półbiegu w km/h 
        self.declare_parameter('gear1_max_speed_kmh', 7.5)   # km/h 
        self.declare_parameter('gear2_max_speed_kmh', 8.7)  # km/h 
        self.declare_parameter('gear3_max_speed_kmh', 10.2)  # km/h 
        self.declare_parameter('gear4_max_speed_kmh', 11.9)  # km/h
        # Parametry max_speed dla każdego półbiegu w km/h
        # self.declare_parameter('gear1_max_speed_kmh', 11.5)   # km/h 
        # self.declare_parameter('gear2_max_speed_kmh', 13.5)  # km/h 
        # self.declare_parameter('gear3_max_speed_kmh', 16.0)  # km/h 
        # self.declare_parameter('gear4_max_speed_kmh', 18.6)  # km/h 
        
        self.declare_parameter('min_speed', 0.5)  # m/s - minimalna prędkość
        self.declare_parameter('max_speed', 8.0)  # m/s - maksymalna prędkość (domyślna, nadpisywana przez gear_max_speeds)
        self.declare_parameter('max_acceleration', 0.5)  # m/s² - maksymalne przyspieszenie
        self.declare_parameter('gps_timeout', 2.0)  # s - timeout sygnału GPS
        self.declare_parameter('control_frequency', 10.0)  # Hz - częstotliwość regulacji
        
        # Pobierz parametry
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.speed_tolerance = self.get_parameter('speed_tolerance').get_parameter_value().double_value
        
        # Parametry PID - domyślne
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        
        # Stan biegów - musi być zainicjalizowany przed init_gear_max_speeds()
        self.current_gear = 0  # Aktualny półbieg (0-4)
        self.clutch_pressed = False  # Stan sprzęgła
        
        # Inicjalizuj parametry PID dla półbiegów
        self.init_gear_pid_params()
        
        # Inicjalizuj parametry max_speed dla półbiegów
        self.init_gear_max_speeds()
        
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.gps_timeout = self.get_parameter('gps_timeout').get_parameter_value().double_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # --- Zmienne stanu ---
        self.is_enabled = False  # Stan włączania/wyłączania autopilota
        self.activation_conditions_checked = False  # Czy warunki aktywacji zostały już sprawdzone
        self.current_distance_longitudinal = 0.0  # Aktualna odległość wzdłużna
        self.tractor_speed = 0.0  # Prędkość ciągnika
        self.harvester_speed = 0.0  # Prędkość sieczkarni
        
        # Zmienne dla regulatora PID
        self.integral_error = 0.0  # Błąd całkujący dla regulatora PID
        self.previous_error = 0.0  # Poprzedni błąd dla członu różniczkującego
        self.last_target_speed = 0.0  # Ostatnia zadana prędkość
        self.last_control_time = time.time()  # Czas ostatniej regulacji
        
        
        # --- Timestamps dla timeoutów ---
        self.last_distance_update = time.time()
        self.last_tractor_gps_update = time.time()
        self.last_harvester_gps_update = time.time()
        self.last_gear_update = time.time()
        
        # --- QoS ---
        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Subskrypcje ---
        self.create_subscription(
            DistanceMetrics, 
            '/distance_metrics', 
            self.distance_callback, 
            default_qos
        )
        
        self.create_subscription(
            GpsRtk, 
            '/gps_rtk_data/tractor_filtered', 
            self.tractor_gps_callback, 
            default_qos
        )
        
        self.create_subscription(
            GpsRtk, 
            '/gps_rtk_data/chopper_filtered', 
            self.harvester_gps_callback, 
            default_qos
        )
        
        # Subskrypcja pozycji zadanej z web interface
        self.create_subscription(
            Float64, 
            '/target_position', 
            self.target_position_callback, 
            default_qos
        )
        
        # Subskrypcja stanu biegów
        self.create_subscription(
            Gear, 
            '/gears', 
            self.gear_callback, 
            default_qos
        )
        
        # --- Publikacje ---
        self.target_speed_publisher = self.create_publisher(Float64, '/target_speed', default_qos)
        self.status_publisher = self.create_publisher(String, '/autopilot/status', default_qos)
        self.health_publisher = self.create_publisher(String, '/position_controller/health', default_qos)
        
        # --- Serwisy ---
        self.set_enabled_service = self.create_service(
            SetBool, 
            '/position_controller/set_enabled', 
            self.set_enabled_callback
        )
        
        self.set_parameters_service = self.create_service(
            SetParameters, 
            'position_controller_node/set_parameters', 
            self.set_parameters_service_callback
        )
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # --- Timery ---
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        # --- Logowanie ---
        self.get_logger().info("=== POSITION CONTROLLER NODE STARTED ===")
        self.get_logger().info(f"Docelowa odległość: {self.target_distance} m")
        self.get_logger().info(f"Tolerancja pozycji: ±{self.position_tolerance} m")
        self.get_logger().info(f"Tolerancja prędkości: ±{self.speed_tolerance} m/s")
        self.get_logger().info(f"Nastawy PID (domyślne): Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        self.get_logger().info(f"Limity prędkości: min={self.min_speed:.1f}m/s, max={self.max_speed:.1f}m/s (aktywny dla biegu {self.current_gear})")
        self.get_logger().info(f"Autopilot domyślnie WYŁĄCZONY")

    def init_gear_pid_params(self):
        """Inicjalizuje parametry PID dla każdego półbiegu."""
        self.gear_pid_params = {}
        
        for gear in range(1, 5):  # Półbiegi 1-4
            self.gear_pid_params[gear] = {
                'Kp': self.get_parameter(f'gear{gear}_Kp').get_parameter_value().double_value,
                'Ki': self.get_parameter(f'gear{gear}_Ki').get_parameter_value().double_value,
                'Kd': self.get_parameter(f'gear{gear}_Kd').get_parameter_value().double_value
            }
        
        self.get_logger().info("Parametry PID dla półbiegów zainicjalizowane")

    def init_gear_max_speeds(self):
        """Inicjalizuje parametry max_speed dla każdego półbiegu."""
        self.gear_max_speeds = {}  # Przechowuje wartości w m/s
        
        for gear in range(1, 5):
            # Pobierz wartość w km/h i konwertuj na m/s
            speed_kmh = self.get_parameter(f'gear{gear}_max_speed_kmh').get_parameter_value().double_value
            self.gear_max_speeds[gear] = speed_kmh / 3.6  # Konwersja km/h -> m/s
        
        if self.current_gear in self.gear_max_speeds:
            self.max_speed = self.gear_max_speeds[self.current_gear]
        
        self.get_logger().info("Parametry max_speed dla półbiegów zainicjalizowane")
        self.get_logger().info(f"Max speeds: Bieg1={self.gear_max_speeds[1]:.1f}m/s ({self.gear_max_speeds[1]*3.6:.1f}km/h), Bieg2={self.gear_max_speeds[2]:.1f}m/s ({self.gear_max_speeds[2]*3.6:.1f}km/h), Bieg3={self.gear_max_speeds[3]:.1f}m/s ({self.gear_max_speeds[3]*3.6:.1f}km/h), Bieg4={self.gear_max_speeds[4]:.1f}m/s ({self.gear_max_speeds[4]*3.6:.1f}km/h)")

    def update_pid_params_for_current_gear(self):
        """Aktualizuje parametry PID na podstawie aktualnego półbiegu."""
        if self.current_gear in self.gear_pid_params:
            old_params = f"Kp={self.Kp:.4f}, Ki={self.Ki:.4f}, Kd={self.Kd:.4f}"
            
            self.Kp = self.gear_pid_params[self.current_gear]['Kp']
            self.Ki = self.gear_pid_params[self.current_gear]['Ki']
            self.Kd = self.gear_pid_params[self.current_gear]['Kd']
            
            new_params = f"Kp={self.Kp:.4f}, Ki={self.Ki:.4f}, Kd={self.Kd:.4f}"
            self.get_logger().info(f"Zmiana parametrów PID dla półbiegu {self.current_gear}: {old_params} → {new_params}")
            
            # Reset integratora przy zmianie parametrów
            self.integral_error = 0.0
            self.previous_error = 0.0
        else:
            self.get_logger().warn(f"Brak parametrów PID dla półbiegu {self.current_gear}, używam domyślnych")

    def update_speed_limits_for_current_gear(self):
        """Aktualizuje limity prędkości na podstawie aktualnego półbiegu."""
        if self.current_gear in self.gear_max_speeds:
            old_max = self.max_speed
            self.max_speed = self.gear_max_speeds[self.current_gear]
            
            self.get_logger().info(
                f"Zmiana max_speed dla półbiegu {self.current_gear}: "
                f"{old_max:.2f} → {self.max_speed:.2f} m/s ({old_max*3.6:.1f} → {self.max_speed*3.6:.1f} km/h)"
            )

        else:
            self.get_logger().warn(f"Brak parametrów max_speed dla półbiegu {self.current_gear}, używam domyślnych")

    def gear_callback(self, msg):
        """Odbiera stan biegów i sprzęgła."""
        self.last_gear_update = time.time()
        
        old_gear = self.current_gear
        self.current_gear = msg.gear
        self.clutch_pressed = (msg.clutch_state == 1)
        
        # Sprawdź czy półbieg się zmienił
        if old_gear != self.current_gear and self.current_gear > 0:
            self.get_logger().info(f"Zmiana półbiegu: {old_gear} → {self.current_gear}")
            self.update_pid_params_for_current_gear()
            self.update_speed_limits_for_current_gear()
        
        if self.is_enabled:
            self.get_logger().debug(f"Półbieg: {self.current_gear}, Sprzęgło: {'wciśnięte' if self.clutch_pressed else 'zwolnione'}")

    def update_parameter_and_variable(self, param_name, value, variable_name):
        try:
            setattr(self, variable_name, value)
            self.get_logger().info(f"Zaktualizowano {param_name} na: {value}")
            return True
        except Exception as e:
            self.get_logger().error(f"Błąd podczas aktualizacji {param_name}: {e}")
            return False

    def parameters_callback(self, params):
        self.get_logger().info("=== OTRZYMANO ZMIANĘ PARAMETRÓW ===")
        
        for param in params:
            if param.name == "Kp":
                self.update_parameter_and_variable('Kp', param.value, 'Kp')
            elif param.name == "Ki":
                self.update_parameter_and_variable('Ki', param.value, 'Ki')
            elif param.name == "Kd":
                self.update_parameter_and_variable('Kd', param.value, 'Kd')
            elif param.name == "position_tolerance":
                self.update_parameter_and_variable('position_tolerance', param.value, 'position_tolerance')
            elif param.name == "speed_tolerance":
                self.update_parameter_and_variable('speed_tolerance', param.value, 'speed_tolerance')
            elif param.name == "target_distance":
                self.update_parameter_and_variable('target_distance', param.value, 'target_distance')
            # Obsługa parametrów dla półbiegów
            elif param.name.startswith("gear") and param.name.endswith("_Kp"):
                gear_num = int(param.name[4:5])
                if gear_num in self.gear_pid_params:
                    old_value = self.gear_pid_params[gear_num]['Kp']
                    self.gear_pid_params[gear_num]['Kp'] = param.value
                    self.get_logger().info(f"Zmiana gear{gear_num}_Kp: {old_value:.4f} → {param.value:.4f}")
                    if self.current_gear == gear_num:
                        self.Kp = param.value
                        self.get_logger().info(f"Zaktualizowano aktywny Kp dla półbiegu {gear_num}")
            elif param.name.startswith("gear") and param.name.endswith("_Ki"):
                gear_num = int(param.name[4:5])
                if gear_num in self.gear_pid_params:
                    old_value = self.gear_pid_params[gear_num]['Ki']
                    self.gear_pid_params[gear_num]['Ki'] = param.value
                    self.get_logger().info(f"Zmiana gear{gear_num}_Ki: {old_value:.4f} → {param.value:.4f}")
                    if self.current_gear == gear_num:
                        self.Ki = param.value
                        self.get_logger().info(f"Zaktualizowano aktywny Ki dla półbiegu {gear_num}")
            elif param.name.startswith("gear") and param.name.endswith("_Kd"):
                gear_num = int(param.name[4:5])
                if gear_num in self.gear_pid_params:
                    old_value = self.gear_pid_params[gear_num]['Kd']
                    self.gear_pid_params[gear_num]['Kd'] = param.value
                    self.get_logger().info(f"Zmiana gear{gear_num}_Kd: {old_value:.4f} → {param.value:.4f}")
                    if self.current_gear == gear_num:
                        self.Kd = param.value
                        self.get_logger().info(f"Zaktualizowano aktywny Kd dla półbiegu {gear_num}")
            # Obsługa parametrów max_speed dla półbiegów (w km/h)
            elif param.name.startswith("gear") and param.name.endswith("_max_speed_kmh"):
                gear_num = int(param.name[4:5])
                if gear_num in self.gear_max_speeds:
                    old_value_ms = self.gear_max_speeds[gear_num]
                    old_value_kmh = old_value_ms * 3.6
                    new_value_ms = param.value / 3.6  # Konwersja km/h -> m/s
                    self.gear_max_speeds[gear_num] = new_value_ms
                    self.get_logger().info(f"Zmiana gear{gear_num}_max_speed_kmh: {old_value_kmh:.1f} → {param.value:.1f} km/h ({old_value_ms:.2f} → {new_value_ms:.2f} m/s)")
                    if self.current_gear == gear_num:
                        self.max_speed = new_value_ms
                        self.get_logger().info(f"Zaktualizowano aktywny max_speed dla półbiegu {gear_num}: {param.value:.1f} km/h ({new_value_ms:.2f} m/s)")
                        # Reset integratora przy zmianie max_speed
                        self.integral_error = 0.0
        
        # Log aktualnych parametrów dla wszystkich półbiegów po zmianie
        self.get_logger().info("=== AKTUALNE PARAMETRY PID I MAX_SPEED DLA WSZYSTKICH PÓŁBIEGÓW ===")
        for gear in range(1, 5):
            if gear in self.gear_pid_params and gear in self.gear_max_speeds:
                pid_params = self.gear_pid_params[gear]
                max_speed = self.gear_max_speeds[gear]
                active_marker = " [AKTYWNY]" if gear == self.current_gear else ""
                self.get_logger().info(
                    f"Półbieg {gear}: Kp={pid_params['Kp']:.4f}, Ki={pid_params['Ki']:.4f}, Kd={pid_params['Kd']:.4f}, "
                    f"max_speed={max_speed:.2f}m/s ({max_speed*3.6:.1f}km/h){active_marker}"
                )
        self.get_logger().info(f"Aktualnie aktywny półbieg: {self.current_gear}")
        self.get_logger().info(f"Aktywne parametry: Kp={self.Kp:.4f}, Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, max_speed={self.max_speed:.2f}m/s")
        self.get_logger().info("=== KONIEC ZMIANY PARAMETRÓW ===")
        
        return SetParametersResult(successful=True)

    def set_parameters_service_callback(self, request, response):
        """Serwis do ustawiania parametrów regulatora pozycji."""
        try:
            success_count = 0
            total_params = len(request.parameters)
            
            for param in request.parameters:
                param_name = param.name
                param_value = param.value.double_value
                
                if param_name == 'Kp':
                    if self.update_parameter_and_variable('Kp', param_value, 'Kp'):
                        success_count += 1
                elif param_name == 'Ki':
                    if self.update_parameter_and_variable('Ki', param_value, 'Ki'):
                        success_count += 1
                elif param_name == 'Kd':
                    if self.update_parameter_and_variable('Kd', param_value, 'Kd'):
                        success_count += 1
                elif param_name == 'position_tolerance':
                    if self.update_parameter_and_variable('position_tolerance', param_value, 'position_tolerance'):
                        success_count += 1
                elif param_name == 'speed_tolerance':
                    if self.update_parameter_and_variable('speed_tolerance', param_value, 'speed_tolerance'):
                        success_count += 1
                elif param_name == 'target_distance':
                    if self.update_parameter_and_variable('target_distance', param_value, 'target_distance'):
                        success_count += 1
                # Obsługa parametrów dla półbiegów
                elif param_name.startswith('gear') and param_name.endswith('_Kp'):
                    gear_num = int(param_name[4:5])
                    if gear_num in self.gear_pid_params:
                        self.gear_pid_params[gear_num]['Kp'] = param_value
                        if self.current_gear == gear_num:
                            self.Kp = param_value
                        success_count += 1
                elif param_name.startswith('gear') and param_name.endswith('_Ki'):
                    gear_num = int(param_name[4:5])
                    if gear_num in self.gear_pid_params:
                        self.gear_pid_params[gear_num]['Ki'] = param_value
                        if self.current_gear == gear_num:
                            self.Ki = param_value
                        success_count += 1
                elif param_name.startswith('gear') and param_name.endswith('_Kd'):
                    gear_num = int(param_name[4:5])
                    if gear_num in self.gear_pid_params:
                        self.gear_pid_params[gear_num]['Kd'] = param_value
                        if self.current_gear == gear_num:
                            self.Kd = param_value
                        success_count += 1
                # Obsługa parametrów max_speed dla półbiegów (w km/h)
                elif param_name.startswith('gear') and param_name.endswith('_max_speed_kmh'):
                    gear_num = int(param_name[4:5])
                    if gear_num in self.gear_max_speeds:
                        # Konwertuj km/h na m/s
                        speed_ms = param_value / 3.6
                        self.gear_max_speeds[gear_num] = speed_ms
                        if self.current_gear == gear_num:
                            self.max_speed = speed_ms
                            # Reset integratora przy zmianie max_speed
                            self.integral_error = 0.0
                        success_count += 1
                else:
                    self.get_logger().warn(f"Nieznany parametr: {param_name}")
            
            # Przygotuj odpowiedź
            response.results = []
            for param in request.parameters:
                result = SetParametersResult()
                result.successful = (
                    param.name in ['Kp', 'Ki', 'Kd', 'position_tolerance', 'speed_tolerance', 'target_distance'] or 
                    (param.name.startswith('gear') and ('_Kp' in param.name or '_Ki' in param.name or '_Kd' in param.name or '_max_speed_kmh' in param.name)) and 
                    success_count > 0
                )
                response.results.append(result)
            
            if success_count == total_params:
                self.get_logger().info("=== PARAMETRY ZAKTUALIZOWANE POMYŚLNIE ===")
                self.get_logger().info(f"Parametry globalne: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, position_tolerance={self.position_tolerance}, speed_tolerance={self.speed_tolerance}, target_distance={self.target_distance}")
                
                # Log nastaw dla wszystkich półbiegów
                self.get_logger().info("=== NASTAWY PID I MAX_SPEED DLA WSZYSTKICH PÓŁBIEGÓW ===")
                for gear in range(1, 5):
                    if gear in self.gear_pid_params and gear in self.gear_max_speeds:
                        pid_params = self.gear_pid_params[gear]
                        max_speed = self.gear_max_speeds[gear]
                        active_marker = " [AKTYWNY]" if gear == self.current_gear else ""
                        self.get_logger().info(
                            f"Półbieg {gear}: Kp={pid_params['Kp']:.4f}, Ki={pid_params['Ki']:.4f}, Kd={pid_params['Kd']:.4f}, "
                            f"max_speed={max_speed:.2f}m/s ({max_speed*3.6:.1f}km/h){active_marker}"
                        )
                self.get_logger().info(f"Aktualnie aktywny półbieg: {self.current_gear}")
                self.get_logger().info(f"Aktywne parametry: Kp={self.Kp:.4f}, Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, max_speed={self.max_speed:.2f}m/s")
                self.get_logger().info("=== KONIEC AKTUALIZACJI PARAMETRÓW ===")
            else:
                self.get_logger().warn(f"Zaktualizowano {success_count}/{total_params} parametrów")
                
            return response
            
        except Exception as e:
            self.get_logger().error(f"Błąd w serwisie ustawiania parametrów: {e}")
            response.results = []
            for param in request.parameters:
                result = SetParametersResult()
                result.successful = False
                response.results.append(result)
            return response

    def distance_callback(self, msg):
        """Odbiera dane o odległościach względnych."""
        self.last_distance_update = time.time()
        self.current_distance_longitudinal = msg.distance_longitudinal
        
        if self.is_enabled:
            self.get_logger().debug(f"Odległość wzdłużna: {self.current_distance_longitudinal:.2f} m")

    def tractor_gps_callback(self, msg):
        """Odbiera dane GPS ciągnika (filtrowane)."""
        self.last_tractor_gps_update = time.time()
        self.tractor_speed = msg.speed_mps
        
        if self.is_enabled:
            self.get_logger().debug(f"Prędkość ciągnika: {self.tractor_speed:.2f} m/s")

    def harvester_gps_callback(self, msg):
        """Odbiera dane GPS sieczkarni."""
        self.last_harvester_gps_update = time.time()
        self.harvester_speed = msg.speed_mps
        
        if self.is_enabled:
            self.get_logger().debug(f"Prędkość sieczkarni: {self.harvester_speed:.2f} m/s")

    def target_position_callback(self, msg):
        """Odbiera pozycję zadaną z web interface."""
        self.target_distance = msg.data
        self.get_logger().info(f"Otrzymano nową pozycję zadaną: {self.target_distance} m")

    def set_enabled_callback(self, request, response):
        """Serwis do włączania/wyłączania autopilota."""
        self.is_enabled = request.data
        
        if self.is_enabled:
            self.get_logger().info("Autopilot WŁĄCZONY")
            self.integral_error = 0.0
            self.activation_conditions_checked = False
        else:
            self.get_logger().info("Autopilot WYŁĄCZONY")
            self.integral_error = 0.0
            self.activation_conditions_checked = False
        
        response.success = True
        response.message = f"Autopilot {'włączony' if self.is_enabled else 'wyłączony'}"
        return response

    def check_activation_conditions(self):
        """Sprawdza warunki aktywacji autopilota."""
        # Sprawdź timeout danych
        current_time = time.time()
        if (current_time - self.last_distance_update > self.gps_timeout or
            current_time - self.last_tractor_gps_update > self.gps_timeout or
            current_time - self.last_harvester_gps_update > self.gps_timeout):
            return False, "Timeout danych GPS"
        
        # Sprawdź warunki pozycji i prędkości
        position_ok = abs(self.current_distance_longitudinal) <= self.position_tolerance
        speed_difference = abs(self.harvester_speed - self.tractor_speed)
        speed_ok = speed_difference <= self.speed_tolerance
        
        if position_ok and speed_ok:
            return True, "Warunki spełnione"
        else:
            return False, f"Pozycja: {position_ok},current_distance_longitudinal: {self.current_distance_longitudinal}, Prędkość: {speed_ok},harvester_speed: {self.harvester_speed}, tractor_speed: {self.tractor_speed}"

    def calculate_pid_control(self, position_error, dt):
        """Oblicza sygnał sterujący regulatora PID."""
        # Człon proporcjonalny
        p_term = self.Kp * position_error
        
        # Człon całkujący
        self.integral_error += position_error * dt
        i_term = self.Ki * self.integral_error
        
        # Człon różniczkujący
        if dt > 0:
            derivative_error = (position_error - self.previous_error) / dt
            d_term = self.Kd * derivative_error
        else:
            d_term = 0.0
        
        # Sygnał sterujący
        control_signal = p_term + i_term + d_term
        
        # Aktualizuj poprzedni błąd
        self.previous_error = position_error
        
        # Zwróć szczegółowe informacje o członach
        return {
            'control_signal': control_signal,
            'p_term': p_term,
            'i_term': i_term,
            'd_term': d_term,
            'integral_error': self.integral_error,
            'derivative_error': derivative_error if dt > 0 else 0.0
        }

    def apply_anti_windup(self, target_speed, dt):
        """Stosuje mechanizm anti-windup."""
        # Sprawdź czy sygnał sterujący jest nasycony
        if target_speed >= self.max_speed:
            # Sygnał nasycony w górę - blokuj integrator jeśli błąd ma ten sam znak
            if self.integral_error > 0:
                self.integral_error -= (target_speed - self.max_speed) * dt / self.Ki
        elif target_speed <= self.min_speed:
            # Sygnał nasycony w dół - blokuj integrator jeśli błąd ma ten sam znak
            if self.integral_error < 0:
                self.integral_error -= (target_speed - self.min_speed) * dt / self.Ki

    def apply_limits(self, target_speed):
        """Stosuje ograniczenia prędkości i przyspieszenia."""
        # Ograniczenia prędkości
        target_speed = max(self.min_speed, min(self.max_speed, target_speed))
        
        # Ograniczenia przyspieszenia
        current_time = time.time()
        dt = current_time - self.last_control_time
        if dt > 0:
            max_speed_change = self.max_acceleration * dt
            speed_change = target_speed - self.last_target_speed
            
            if abs(speed_change) > max_speed_change:
                if speed_change > 0:
                    target_speed = self.last_target_speed + max_speed_change
                else:
                    target_speed = self.last_target_speed - max_speed_change
        
        self.last_target_speed = target_speed
        self.last_control_time = current_time
        
        return target_speed

    def control_loop(self):
        """Główna pętla regulacji pozycji."""
        if not self.is_enabled:
            return
        
        # Sprawdź warunki aktywacji
        if not self.activation_conditions_checked:
            conditions_ok, condition_msg = self.check_activation_conditions()
            if not conditions_ok:
                self.get_logger().warn(f"Autopilot nieaktywny: {condition_msg}")
                return
            else:
                self.get_logger().info(f"Warunki aktywacji spełnione: {condition_msg}")
                self.activation_conditions_checked = True
        
        # Oblicz błąd pozycji
        position_error = self.target_distance - self.current_distance_longitudinal
        speed_correction = self.Kp * position_error
        
        # Oblicz docelową prędkość z feed-forward
        target_speed = self.harvester_speed + speed_correction

        target_speed = self.apply_limits(target_speed)
        
        # Publikuj docelową prędkość
        speed_msg = Float64()
        speed_msg.data = target_speed
        self.target_speed_publisher.publish(speed_msg)
            
        

    def publish_status(self, status, message):
        """Publikuje status autopilota."""
        status_msg = String()
        status_msg.data = f"{status}: {message}"
        self.status_publisher.publish(status_msg)

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            current_time = time.time()
            
            # Sprawdź timeouty
            distance_timeout = current_time - self.last_distance_update > self.gps_timeout
            tractor_timeout = current_time - self.last_tractor_gps_update > self.gps_timeout
            harvester_timeout = current_time - self.last_harvester_gps_update > self.gps_timeout
            gear_timeout = current_time - self.last_gear_update > self.gps_timeout
            
            # Przygotuj dane health
            health_data = {
                'node_name': 'position_controller_node',
                'is_enabled': self.is_enabled,
                'distance_timeout': distance_timeout,
                'tractor_timeout': tractor_timeout,
                'harvester_timeout': harvester_timeout,
                'gear_timeout': gear_timeout,
                'current_distance': self.current_distance_longitudinal,
                'tractor_speed': self.tractor_speed,
                'harvester_speed': self.harvester_speed,
                'current_gear': self.current_gear,
                'clutch_pressed': self.clutch_pressed,
                'pid_params': {'Kp': self.Kp, 'Ki': self.Ki, 'Kd': self.Kd},
                'integral_error': self.integral_error,
                'previous_error': self.previous_error
            }
            
            # Publikuj health
            health_msg = String()
            health_msg.data = str(health_data)
            self.health_publisher.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd publikacji health: {e}")

def main():
    rclpy.init()
    node = PositionControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
