# servo_controller/servo_controller/servo_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from my_robot_interfaces.msg import StampedInt32
from std_srvs.srv import SetBool
import time
import json
import psutil
from std_msgs.msg import String
from adafruit_servokit import ServoKit

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Parametr dla watchdoga
        self.declare_parameter('watchdog_timeout_sec', 0.2)
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').get_parameter_value().double_value

        # Inicjalizacja sprzętu
        try:
            self.kit = ServoKit(channels=16)
        except Exception as e:
            self.get_logger().error(f"Nie udało się zainicjować ServoKit (PCA9685): {e}")
            rclpy.shutdown()
            return
        self.servo_channel = 0
        self.kit.servo[self.servo_channel].set_pulse_width_range(500, 2500)
        
        # QoS i subskrypcja
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            StampedInt32,
            'servo/set_angle',
            self.set_target_angle_callback,
            qos_profile=sensor_qos_profile
        )

        # Publisher i timery
        self.position_publisher = self.create_publisher(StampedInt32, 'servo/position', qos_profile=sensor_qos_profile)
        self.SERVO_SPEED_DEGPS = 750.0 # Prędkość serwa w stopniach/sekundę
        movement_timer_period = 1.0 / 100.0 # 100 Hz
        self.movement_timer = self.create_timer(movement_timer_period, self.movement_step_callback)
        publish_timer_period = 1.0 / 50.0 # 50 Hz
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_position_callback)

        # Logika Watchdoga
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        watchdog_timer_period = 0.2 # Sprawdzaj 5 razy na sekundę
        self.watchdog_timer = self.create_timer(watchdog_timer_period, self.watchdog_callback)

        # Zmienne stanu serwa
        self.current_simulated_angle = 0.0
        self.target_angle = 0.0
        self.manual_mode = False  # NOWA ZMIENNA: tryb ręczny

        self.set_mode_service = self.create_service(SetBool, '/servo/set_manual_mode', self.set_manual_mode_callback)

        self.health_pub = self.create_publisher(String, '/mss/node_health/servo_controller', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)

        self.get_logger().info('Servo controller node started.')
        self.get_logger().info(f"Watchdog aktywny. Timeout: {self.watchdog_timeout}s.")
        self.get_logger().info("Tryb automatyczny (watchdog aktywny). Użyj service '/servo/set_manual_mode' aby przełączyć.")
        self.set_initial_angle(0)

    def set_manual_mode_callback(self, request, response):
        """Service callback do przełączania trybu serwa."""
        self.get_logger().info(f"Otrzymano żądanie zmiany trybu serwa: {request.data}")
        
        self.manual_mode = request.data
        
        if self.manual_mode:
            self.get_logger().info("PRZEŁĄCZONO NA TRYB RĘCZNY - watchdog wyłączony!")
            response.message = "Tryb ręczny włączony - watchdog wyłączony"
        else:
            self.get_logger().info("PRZEŁĄCZONO NA TRYB AUTOMATYCZNY - watchdog włączony!")
            response.message = "Tryb automatyczny włączony - watchdog włączony"
        
        response.success = True
        self.get_logger().info(f"Service zwraca: success={response.success}, message={response.message}")
        return response

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status sprzętu
            hardware_status = "OK"
            try:
                # Sprawdź czy ServoKit działa
                self.kit.servo[self.servo_channel].angle
                hardware_status = "OK"
            except Exception:
                hardware_status = "ERROR"
            
            # Sprawdź status timerów
            timers_status = "OK"
            if not hasattr(self, 'movement_timer') or not hasattr(self, 'publish_timer'):
                timers_status = "ERROR"
            
            # Sprawdź status watchdoga
            watchdog_status = "ACTIVE" if not self.manual_mode else "DISABLED"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if hardware_status == "ERROR":
                errors.append("Problem z sprzętem serwa")
            if timers_status == "ERROR":
                errors.append("Timery nieaktywne")
            if self.manual_mode:
                warnings.append("Tryb ręczny - watchdog wyłączony")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'hardware_status': hardware_status,
                'timers_status': timers_status,
                'watchdog_status': watchdog_status,
                'manual_mode': self.manual_mode,
                'current_angle': self.current_simulated_angle,
                'target_angle': self.target_angle,
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

    def set_target_angle_callback(self, msg):
        # Resetujemy czas watchdoga przy każdej nowej komendzie
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        
        target_angle = msg.data
        if 0 <= target_angle <= 180:
            self.target_angle = float(target_angle)
        else:
            self.get_logger().warn(f"Received angle {target_angle} out of bounds (0-180). Ignoring command.")

    def watchdog_callback(self):
        """Sprawdza, czy nie upłynął czas od ostatniej wiadomości."""
        if self.manual_mode:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_msg = current_time - self.last_msg_time

        if time_since_last_msg > self.watchdog_timeout:
            if self.target_angle != 0.0:
                self.get_logger().warn(f"Watchdog timeout! Brak komendy przez >{self.watchdog_timeout:.1f}s. Ustawiam kąt na 0.")
                self.target_angle = 0.0
                self.last_msg_time = current_time

    def set_initial_angle(self, angle: int):
        safe_angle = float(max(0, min(180, angle)))
        self.current_simulated_angle = safe_angle
        self.target_angle = safe_angle
        self.kit.servo[self.servo_channel].angle = int(safe_angle)
        
    def movement_step_callback(self):
        """Symuluje i wykonuje płynny ruch serwa do pozycji docelowej."""
        error = self.target_angle - self.current_simulated_angle
        if abs(error) < 0.01:
            return

        # Oblicz maksymalny ruch w tym kroku czasowym
        tick_duration = self.movement_timer.timer_period_ns / 1e9
        max_step = self.SERVO_SPEED_DEGPS * tick_duration
        
        # Ustaw kierunek ruchu
        step = max_step if error > 0 else -max_step
        
        # Jeśli krok jest większy niż pozostały błąd, wykonaj tylko ruch do celu
        if abs(error) < abs(step):
            step = error
            
        self.current_simulated_angle += step
        self.kit.servo[self.servo_channel].angle = int(round(self.current_simulated_angle))

    def publish_position_callback(self):
        """Publikuje aktualną (symulowaną) pozycję serwa."""
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = int(round(self.current_simulated_angle))
        self.position_publisher.publish(msg)

    def set_angle_for_shutdown(self, angle_degrees: int):
        try:
            self.kit.servo[self.servo_channel].angle = angle_degrees
            time.sleep(0.5)
        except Exception as e:
            print(f"Error setting servo angle to {angle_degrees} during shutdown: {e}")

    def destroy_node(self):
        if hasattr(self, 'kit'):
            self.get_logger().info("Wywołano destroy_node. Ustawiam serwo na 0 dla pewności.")
            self.set_angle_for_shutdown(0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = None
    try:
        servo_controller_node = ServoController()
        if rclpy.ok() and hasattr(servo_controller_node, 'kit'): 
            rclpy.spin(servo_controller_node)
    except KeyboardInterrupt:
        if servo_controller_node:
            servo_controller_node.get_logger().info('Keyboard Interrupt! Shutting down...')
    finally:
        if servo_controller_node:
            servo_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()