import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Importuj wszystkie potrzebne wiadomości
from std_msgs.msg import Float64, String
import json
import psutil
import time
from my_robot_interfaces.msg import GpsRtk, Gear, DistanceMetrics, StampedInt32, DiagnosticData

# Wartości zastępcze
PLACEHOLDER_UINT8 = 255 
PLACEHOLDER_INT = 99999
PLACEHOLDER_FLOAT = 99999.0

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        # --- Parametry ---
        self.declare_parameter('publish_frequency_hz', 5.0)
        self.declare_parameter('data_timeout_sec', 2.0)

        publish_frequency = self.get_parameter('publish_frequency_hz').get_parameter_value().double_value
        self.data_timeout = self.get_parameter('data_timeout_sec').get_parameter_value().double_value

        # --- Zmienne do przechowywania ostatnich wiadomości ---
        self.last_tractor_gps = None
        self.last_chopper_gps = None
        self.last_servo_pos = None
        self.last_gear = None
        self.last_target_speed = None
        self.last_relative_pos = None

        # --- QoS ---
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        target_speed_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # --- Publisher ---
        self.diag_publisher = self.create_publisher(DiagnosticData, '/diagnostics', 10)

        # --- Subskrybenci ---
        self.create_subscription(GpsRtk, '/gps_rtk_data/tractor_filtered', self.tractor_gps_callback, qos_profile)
        self.create_subscription(GpsRtk, '/gps_rtk_data/chopper_filtered', self.chopper_gps_callback, qos_profile)
        self.create_subscription(StampedInt32, '/servo/position', self.servo_pos_callback, qos_profile)
        self.create_subscription(Gear, '/gears', self.gear_callback, qos_profile)
        self.create_subscription(Float64, '/target_speed', self.target_speed_callback, target_speed_qos)  # NAPRAWA: Używamy target_speed_qos
        self.create_subscription(DistanceMetrics, '/distance_metrics', self.relative_pos_callback, qos_profile)

        self.timer = self.create_timer(1.0 / publish_frequency, self.main_loop_callback)
        self.health_pub = self.create_publisher(String, '/mss/node_health/diagnostics_node', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        self.get_logger().info(f'Węzeł diagnostyczny uruchomiony. Publikuje z f={publish_frequency} Hz.')

    # --- Definicje callbacków ---
    def tractor_gps_callback(self, msg): self.last_tractor_gps = msg
    def chopper_gps_callback(self, msg): self.last_chopper_gps = msg
    def servo_pos_callback(self, msg): self.last_servo_pos = msg
    def gear_callback(self, msg): self.last_gear = msg
    def target_speed_callback(self, msg): self.last_target_speed = msg
    def relative_pos_callback(self, msg): self.last_relative_pos = msg

    def is_data_fresh(self, msg):
        if msg is None: return False
        if not hasattr(msg, 'header'):
            return True
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        msg_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        return (current_time_sec - msg_time_sec) < self.data_timeout

    def main_loop_callback(self):
        diag_msg = DiagnosticData()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        # 1. Dane z ciągnika (filtrowane)
        if self.is_data_fresh(self.last_tractor_gps):
            diag_msg.tractor_gps_filtered = self.last_tractor_gps
        else:
            diag_msg.tractor_gps_filtered.rtk_status = PLACEHOLDER_UINT8
            diag_msg.tractor_gps_filtered.latitude_deg = PLACEHOLDER_FLOAT
            diag_msg.tractor_gps_filtered.longitude_deg = PLACEHOLDER_FLOAT
            diag_msg.tractor_gps_filtered.speed_mps = PLACEHOLDER_FLOAT

        # 2. Dane z sieczkarni i status BT
        if self.is_data_fresh(self.last_chopper_gps):
            diag_msg.chopper_gps = self.last_chopper_gps
            diag_msg.bt_status = True
        else:
            diag_msg.chopper_gps.rtk_status = PLACEHOLDER_UINT8
            diag_msg.chopper_gps.latitude_deg = PLACEHOLDER_FLOAT
            diag_msg.chopper_gps.longitude_deg = PLACEHOLDER_FLOAT
            diag_msg.bt_status = False

        # 3. Dane od serwa
        if self.is_data_fresh(self.last_servo_pos):
            diag_msg.servo_position = self.last_servo_pos
        else:
            diag_msg.servo_position.data = PLACEHOLDER_INT
        
        # 4. Dane o biegach
        if self.is_data_fresh(self.last_gear):
            diag_msg.tractor_gear = self.last_gear
        else:
            diag_msg.tractor_gear.gear = PLACEHOLDER_UINT8
            diag_msg.tractor_gear.clutch_state = PLACEHOLDER_UINT8

        # 5. Prędkość zadana
        if self.is_data_fresh(self.last_target_speed):
            diag_msg.target_speed = self.last_target_speed
        else:
            diag_msg.target_speed.data = PLACEHOLDER_FLOAT

        # 6. Pozycja względna
        if self.is_data_fresh(self.last_relative_pos):
            diag_msg.relative_position = self.last_relative_pos
        else:
            diag_msg.relative_position.distance_longitudinal = PLACEHOLDER_FLOAT
            diag_msg.relative_position.distance_lateral = PLACEHOLDER_FLOAT

        self.diag_publisher.publish(diag_msg)

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status timerów
            timer_status = "OK" if hasattr(self, 'timer') else "ERROR"
            
            # Sprawdź status publisherów
            publisher_status = "OK" if hasattr(self, 'diag_publisher') else "ERROR"
            
            # Sprawdź status subskrypcji
            subscription_status = "OK"
            # Sprawdź czy istnieją subskrypcje (diagnostics_node ma wiele subskrypcji)
            if not hasattr(self, 'tractor_gps_callback') or not hasattr(self, 'chopper_gps_callback'):
                subscription_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if timer_status == "ERROR":
                errors.append("Timer nieaktywny")
            if publisher_status == "ERROR":
                errors.append("Publisher nieaktywny")
            if subscription_status == "ERROR":
                errors.append("Subskrypcje nieaktywne")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'timer_status': timer_status,
                'publisher_status': publisher_status,
                'subscription_status': subscription_status,
                'publish_frequency_hz': self.get_parameter('publish_frequency_hz').get_parameter_value().double_value,
                'data_timeout_sec': self.data_timeout,
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

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
