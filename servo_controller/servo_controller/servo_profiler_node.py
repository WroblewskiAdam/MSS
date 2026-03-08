# servo_controller/servo_controller/servo_profiler_node.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import StampedInt32
import time
import threading

class ServoProfilerNode(Node):
    def __init__(self):
        super().__init__('servo_profiler_node')

        # Parametry profilowania
        self.declare_parameter('start_angle', 0)
        self.declare_parameter('end_angle', 150)
        self.declare_parameter('angle_step', 5)
        self.declare_parameter('hold_duration_sec', 5.0)
        self.declare_parameter('initial_delay_sec', 1.0)

        self.start_angle = self.get_parameter('start_angle').get_parameter_value().integer_value
        self.end_angle = self.get_parameter('end_angle').get_parameter_value().integer_value
        self.angle_step = self.get_parameter('angle_step').get_parameter_value().integer_value
        self.hold_duration = self.get_parameter('hold_duration_sec').get_parameter_value().double_value
        self.initial_delay = self.get_parameter('initial_delay_sec').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            StampedInt32, 'servo/set_angle', 10
        )

        
        self.target_angle = 0
        self.lock = threading.Lock()

        timer_period = 1.0 / 50.0  # 50 Hz
        self.publish_timer = self.create_timer(timer_period, self.publish_callback)

        self.get_logger().info("Węzeł profilera serwa uruchomiony z publikacją 50Hz.")
        
        # Wątek sterujący sekwencją (zmienia tylko self.target_angle)
        self.profiling_thread = threading.Thread(target=self.run_profiling_sequence, daemon=True)
        self.profiling_thread.start()

    def publish_callback(self):
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.data = int(self.target_angle)
        self.publisher_.publish(msg)

    def run_profiling_sequence(self):
        self.get_logger().info(f"Rozpoczynam sekwencję profilowania od {self.start_angle} do {self.end_angle} stopni...")
        time.sleep(self.initial_delay)
        
        current_angle_step = self.start_angle
        while current_angle_step <= self.end_angle and rclpy.ok():
            self.get_logger().info(f"Ustawiam kąt docelowy na: {current_angle_step} stopni (na {self.hold_duration}s).")
            
            with self.lock:
                self.target_angle = float(current_angle_step)
            
            time.sleep(self.hold_duration)
            
            current_angle_step += self.angle_step

        if rclpy.ok():
            self.get_logger().info("Sekwencja profilowania zakończona. Zamykanie węzła.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoProfilerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n[INFO] Zamykanie węzła profilera.")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Servo Profiler zakończył działanie.")


if __name__ == '__main__':
    main()
