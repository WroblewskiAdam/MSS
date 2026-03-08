# servo_controller/servo_controller/servo_random_jump_node.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import StampedInt32
import time
import threading
import random
import math

class ServoRandomJumpNode(Node):
    def __init__(self):
        super().__init__('servo_random_jump_node')

        # === Parametry generowania losowego ===
        self.declare_parameter('min_angle', 0)
        self.declare_parameter('max_angle', 120)
        self.declare_parameter('max_jump', 50)
        self.declare_parameter('min_jump', 5) 
        self.declare_parameter('min_duration', 6.0) 
        self.declare_parameter('max_duration', 10.0) 
        self.declare_parameter('duration_dt', 1.0)  
        self.declare_parameter('initial_delay_sec', 4.0)

        # Pobieranie parametrów
        self.min_angle = self.get_parameter('min_angle').get_parameter_value().integer_value
        self.max_angle = self.get_parameter('max_angle').get_parameter_value().integer_value
        self.max_jump = self.get_parameter('max_jump').get_parameter_value().integer_value
        self.min_jump = self.get_parameter('min_jump').get_parameter_value().integer_value 
        self.min_duration = self.get_parameter('min_duration').get_parameter_value().double_value
        self.max_duration = self.get_parameter('max_duration').get_parameter_value().double_value
        self.duration_dt = self.get_parameter('duration_dt').get_parameter_value().double_value
        self.initial_delay = self.get_parameter('initial_delay_sec').get_parameter_value().double_value

        # === Walidacja parametrów ===
        if self.min_angle >= self.max_angle:
             self.get_logger().fatal("Parametr 'min_angle' musi być mniejszy niż 'max_angle'. Zamykanie.")
             rclpy.shutdown()
             return
        if self.min_duration >= self.max_duration:
             self.get_logger().fatal("Parametr 'min_duration' musi być mniejszy niż 'max_duration'. Zamykanie.")
             rclpy.shutdown()
             return
        if self.duration_dt <= 0:
             self.get_logger().fatal("Parametr 'duration_dt' musi być dodatni. Zamykanie.")
             rclpy.shutdown()
             return
        if self.min_jump < 0:
             self.get_logger().fatal("Parametr 'min_jump' nie może być ujemny. Zamykanie.")
             rclpy.shutdown()
             return
        if self.min_jump > self.max_jump:
             self.get_logger().fatal("Parametr 'min_jump' musi być mniejszy lub równy 'max_jump'. Zamykanie.")
             rclpy.shutdown()
             return
        
        self.get_logger().info("Konfiguracja generatora losowych skoków:")
        self.get_logger().info(f"  Zakres kątów:   [{self.min_angle}, {self.max_angle}]")
        self.get_logger().info(f"  Maksymalny skok: {self.max_jump}")
        self.get_logger().info(f"  Minimalny skok:  {self.min_jump}")
        self.get_logger().info(f"  Zakres czasu:    [{self.min_duration}, {self.max_duration}] (krok: {self.duration_dt}s)")


        self.publisher_ = self.create_publisher(
            StampedInt32, 'servo/set_angle', 10
        )

        self.target_angle = float((self.min_angle + self.max_angle) // 2)
        self.lock = threading.Lock()

        timer_period = 1.0 / 10.0  # 10 Hz
        self.publish_timer = self.create_timer(timer_period, self.publish_callback)

        self.get_logger().info("Węzeł losowych skoków serwa uruchomiony (publikacja 50Hz).")
        
        self.jump_thread = threading.Thread(target=self.run_jump_sequence, daemon=True)
        self.jump_thread.start()

    def publish_callback(self):
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.data = int(self.target_angle)
        self.publisher_.publish(msg)

    def get_random_duration(self) -> float:
        """Oblicza losowy czas trwania będący wielokrotnością duration_dt."""
        min_steps = math.ceil(self.min_duration / self.duration_dt)
        max_steps = math.floor(self.max_duration / self.duration_dt)

        if min_steps > max_steps:
            self.get_logger().warn(f"Nie można wygenerować czasu (dt={self.duration_dt} jest zbyt duże dla zakresu). Używam min_duration.")
            return self.min_duration

        random_steps = random.randint(int(min_steps), int(max_steps))
        return random_steps * self.duration_dt

    def get_next_random_angle(self, current_angle: int) -> int:
        min_jump_allowed = max(-self.max_jump, self.min_angle - current_angle)
        max_jump_allowed = min(self.max_jump, self.max_angle - current_angle)

        if min_jump_allowed > max_jump_allowed:
            return current_angle 

        # 2. Zbuduj listę "dozwolonych zakresów" z uwzględnieniem min_jump
        possible_ranges = []
        
        # Zakres 1: Skoki ujemne (muszą być <= -min_jump)
        neg_range_end = min(max_jump_allowed, -self.min_jump)
        if min_jump_allowed <= neg_range_end:
            possible_ranges.append((min_jump_allowed, neg_range_end))
            
        # Zakres 2: Skoki dodatnie (muszą być >= min_jump)
        pos_range_start = max(min_jump_allowed, self.min_jump)
        if pos_range_start <= max_jump_allowed:
            possible_ranges.append((pos_range_start, max_jump_allowed))

        # 3. Wybierz skok
        jump = 0
        if possible_ranges:
            # Udało się znaleźć zakresy spełniające min_jump
            chosen_range = random.choice(possible_ranges)
            jump = random.randint(chosen_range[0], chosen_range[1])
        else:
            self.get_logger().debug(f"Nie można wykonać skoku >= {self.min_jump}. Próbuję wykonać mniejszy ruch.")
            
            if min_jump_allowed <= max_jump_allowed:
                for _ in range(5): # Próbuj wylosować ruch różny od zera
                    jump = random.randint(min_jump_allowed, max_jump_allowed)
                    if jump != 0:
                        break

        return current_angle + jump


    def run_jump_sequence(self):
        self.get_logger().info(f"Rozpoczynam sekwencję losowych skoków za {self.initial_delay}s...")
        time.sleep(self.initial_delay)
        
        current_angle = int(self.target_angle)
        self.get_logger().info(f"Ustawiam pozycję startową: {current_angle} stopni.")
        
        while rclpy.ok():
            next_angle = self.get_next_random_angle(current_angle)
            
            duration = self.get_random_duration()
            
            if next_angle != current_angle:
                self.get_logger().info(f"Nowy cel: {next_angle} stopni (skok: {next_angle-current_angle}, czas: {duration:.2f}s).")
                with self.lock:
                    self.target_angle = float(next_angle)
            else:
                 self.get_logger().info(f"Brak możliwego ruchu. Pozostaję na {current_angle} stopni (na {duration:.2f}s).")

            time.sleep(duration)
            
            current_angle = next_angle

        self.get_logger().info("Sekwencja losowych skoków zatrzymana.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoRandomJumpNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n[INFO] Zamykanie węzła losowych skoków.")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Servo Random Jump zakończył działanie.")


if __name__ == '__main__':
    main()