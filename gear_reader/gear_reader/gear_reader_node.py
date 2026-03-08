import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import Gear
from std_msgs.msg import String
import lgpio
import psutil
import time

# --- Konfiguracja pinów GPIO ---
PIN_CLUTCH = 5
PIN_GEAR_1 = 19
PIN_GEAR_2 = 13
PIN_GEAR_3 = 26
PIN_GEAR_4 = 6

class GearReaderNode(Node):
    def __init__(self):
        super().__init__('gear_reader_node')
        self.get_logger().info("Uruchamianie węzła odczytu biegów (biblioteka lgpio)...")

        self.gear_pins = {
            1: PIN_GEAR_1,
            2: PIN_GEAR_2,
            3: PIN_GEAR_3,
            4: PIN_GEAR_4,
        }
        
        self.chip_handle = None
        self.setup_gpio()

        self.publisher_ = self.create_publisher(Gear, 'gears', 10)
        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_reader_node', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Węzeł gotowy. Publikuje na topiku /gears.")

    def setup_gpio(self):
        try:
            self.chip_handle = lgpio.gpiochip_open(4)
            flags = lgpio.SET_PULL_DOWN
            lgpio.gpio_claim_input(self.chip_handle, PIN_CLUTCH, flags)
            for pin in self.gear_pins.values():
                lgpio.gpio_claim_input(self.chip_handle, pin, flags)
                
            self.get_logger().info("Skonfigurowano piny GPIO przy użyciu lgpio.")
        except lgpio.error as e:
            self.get_logger().error(f"Błąd inicjalizacji GPIO z lgpio: {e}")
            rclpy.shutdown()

    def read_gear_state(self):
        """Odczytuje, który bieg jest aktywny."""
        try:
            for gear, pin in self.gear_pins.items():
                if lgpio.gpio_read(self.chip_handle, pin) == 1:
                    return gear
            return 0
        except lgpio.error as e:
            self.get_logger().warn(f"Błąd odczytu stanu biegu: {e}")
            return 0

    def read_clutch_state(self):
        """Odczytuje stan sprzęgła."""
        try:
            return lgpio.gpio_read(self.chip_handle, PIN_CLUTCH)
        except lgpio.error as e:
            self.get_logger().warn(f"Błąd odczytu stanu sprzęgła: {e}")
            return 0

    def timer_callback(self):
        msg = Gear()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.gear = self.read_gear_state()
        msg.clutch_state = self.read_clutch_state()
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.chip_handle is not None:
            lgpio.gpiochip_close(self.chip_handle)
            self.get_logger().info("Zasoby lgpio zostały zwolnione.")
        super().destroy_node()

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            gpio_status = "OK"
            try:
                lgpio.gpio_read(self.chip_handle, PIN_CLUTCH)
            except Exception:
                gpio_status = "ERROR"
            
            errors = []
            if gpio_status == "ERROR":
                errors.append("Problem z GPIO")
            
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'gpio_status': gpio_status,
                'clutch_pin': PIN_CLUTCH,
                'gear_pins': list(self.gear_pins.values()),
                'errors': errors,
                'warnings': [],
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent
            }
            
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikowania health status: {e}")


def main(args=None):
    rclpy.init(args=args)
    gear_reader_node = GearReaderNode()
    try:
        rclpy.spin(gear_reader_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            gear_reader_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()