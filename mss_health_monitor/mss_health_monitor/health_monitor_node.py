#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import json
import time
from datetime import datetime

from std_msgs.msg import String

class MSSHealthMonitorNode(Node):
    """
    Węzeł monitorujący zdrowie systemu MSS.
    Zbiera raporty health od wszystkich węzłów i publikuje ogólny status systemu.
    """
    
    def __init__(self):
        super().__init__('mss_health_monitor_node')
        
        # Parametry
        self.declare_parameter('health_check_interval', 5.0)
        self.declare_parameter('node_timeout_sec', 15.0)
        
        self.health_check_interval = self.get_parameter('health_check_interval').get_parameter_value().double_value
        self.node_timeout_sec = self.get_parameter('node_timeout_sec').get_parameter_value().double_value
        
        # Lista monitorowanych węzłów
        self.monitored_nodes = [
            'gps_rtk_node',
            'bt_receiver_node',
            'gear_reader_node',
            'gps_mockup_node',          
            'gear_mockup_node',        
            'servo_controller',
            'gear_shifter',
            'tractor_filter_node', 
            'chopper_filter_node', 
            'speed_controller_node',
            'relative_computer_node',
            'gear_manager_node',
            'diagnostics_node',
            'system_monitor',           
            'mss_health_monitor_node'       
        ]
        
        # Stan węzłów
        self.node_states = {}
        self.node_health_data = {}
        self.last_health_updates = {}
        
        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishery
        self.system_status_pub = self.create_publisher(String, '/mss/system_status', qos_profile)
        self.node_status_pub = self.create_publisher(String, '/mss/node_status', qos_profile)
        self.health_alerts_pub = self.create_publisher(String, '/mss/health_alerts', qos_profile)
        
        self.health_pub = self.create_publisher(String, '/mss/node_health/mss_health_monitor_node', qos_profile)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        # Subskrypcje na health status węzłów
        for node_name in self.monitored_nodes:
            topic_name = f'/mss/node_health/{node_name}'
            if node_name == 'gps_rtk_node':
                self.create_subscription(String, topic_name, self.gps_rtk_health_callback, qos_profile)
            elif node_name == 'bt_receiver_node':
                self.create_subscription(String, topic_name, self.bt_receiver_health_callback, qos_profile)
            elif node_name == 'gear_reader_node':
                self.create_subscription(String, topic_name, self.gear_reader_health_callback, qos_profile)
            elif node_name == 'servo_controller':
                self.create_subscription(String, topic_name, self.servo_controller_health_callback, qos_profile)
            elif node_name == 'gear_shifter':
                self.create_subscription(String, topic_name, self.gear_shifter_health_callback, qos_profile)
            elif node_name == 'tractor_filter_node':
                self.create_subscription(String, topic_name, self.tractor_filter_health_callback, qos_profile)
            elif node_name == 'chopper_filter_node':
                self.create_subscription(String, topic_name, self.chopper_filter_health_callback, qos_profile)
            elif node_name == 'speed_controller_node':
                self.create_subscription(String, topic_name, self.speed_controller_health_callback, qos_profile)
            elif node_name == 'relative_computer_node':
                self.create_subscription(String, topic_name, self.relative_computer_health_callback, qos_profile)
            elif node_name == 'gear_manager_node':
                self.create_subscription(String, topic_name, self.gear_manager_health_callback, qos_profile)
            elif node_name == 'diagnostics_node':
                self.create_subscription(String, topic_name, self.diagnostics_health_callback, qos_profile)
            elif node_name == 'system_monitor':
                self.create_subscription(String, topic_name, self.system_monitor_health_callback, qos_profile)
            elif node_name == 'mss_health_monitor_node':
                self.create_subscription(String, topic_name, self.health_monitor_health_callback, qos_profile)
        
        # Timer do publikacji statusu systemu
        self.status_timer = self.create_timer(
            self.health_check_interval, 
            self.publish_system_status
        )
        
        # Inicjalizacja stanu węzłów
        for node_name in self.monitored_nodes:
            self.node_states[node_name] = 'UNKNOWN'
            self.node_health_data[node_name] = {}
            self.last_health_updates[node_name] = 0.0
        
        self.get_logger().info(f"Health Monitor MSS uruchomiony. Monitoruje {len(self.monitored_nodes)} węzłów.")
        self.get_logger().info(f"Interwał sprawdzania: {self.health_check_interval}s")
        self.get_logger().info(f"Timeout węzła: {self.node_timeout_sec}s")
    
    def node_health_callback(self, msg, node_name):
        """Callback dla health status węzłów."""
        try:
            health_data = json.loads(msg.data)
            self.node_states[node_name] = health_data.get('status', 'UNKNOWN')
            self.node_health_data[node_name] = health_data
            self.last_health_updates[node_name] = time.time()
            
            # Logowanie błędów i ostrzeżeń
            errors = health_data.get('errors', [])
            warnings = health_data.get('warnings', [])
            
            if errors:
                self.get_logger().warn(f"Węzeł {node_name} zgłosił błędy: {errors}")
            if warnings:
                self.get_logger().info(f"Węzeł {node_name} zgłosił ostrzeżenia: {warnings}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Nieprawidłowy format JSON od węzła {node_name}")
            self.node_states[node_name] = 'ERROR'

    
    def gps_rtk_health_callback(self, msg):
        self.node_health_callback(msg, 'gps_rtk_node')
    
    def bt_receiver_health_callback(self, msg):
        self.node_health_callback(msg, 'bt_receiver_node')
    
    def gear_reader_health_callback(self, msg):
        self.node_health_callback(msg, 'gear_reader_node')
    
    def servo_controller_health_callback(self, msg):
        self.node_health_callback(msg, 'servo_controller')
    
    def gear_shifter_health_callback(self, msg):
        self.node_health_callback(msg, 'gear_shifter')
    
    def tractor_filter_health_callback(self, msg):
        self.node_health_callback(msg, 'tractor_filter_node')
    
    def chopper_filter_health_callback(self, msg):
        self.node_health_callback(msg, 'chopper_filter_node')
    
    def speed_controller_health_callback(self, msg):
        self.node_health_callback(msg, 'speed_controller_node')
    
    def relative_computer_health_callback(self, msg):
        self.node_health_callback(msg, 'relative_computer_node')
    
    def gear_manager_health_callback(self, msg):
        self.node_health_callback(msg, 'gear_manager_node')
    
    def diagnostics_health_callback(self, msg):
        self.node_health_callback(msg, 'diagnostics_node')
    
    def system_monitor_health_callback(self, msg):
        self.node_health_callback(msg, 'system_monitor')
    
    def health_monitor_health_callback(self, msg):
        self.node_health_callback(msg, 'mss_health_monitor_node')
    
    def check_node_timeouts(self):
        """Sprawdza timeout węzłów na podstawie ostatnich raportów health."""
        current_time = time.time()
        
        for node_name in self.monitored_nodes:
            if node_name in self.last_health_updates:
                time_since_update = current_time - self.last_health_updates[node_name]
                
                if time_since_update > self.node_timeout_sec:
                    if self.node_states[node_name] != 'TIMEOUT':
                        self.node_states[node_name] = 'TIMEOUT'
                        self.get_logger().warn(f"Węzeł {node_name} - timeout ({time_since_update:.1f}s)")
    
    def publish_system_status(self):
        """Publikuje ogólny status systemu."""
        # Sprawdź timeout węzłów
        self.check_node_timeouts()
        
        # Oblicz ogólny status
        total_nodes = len(self.monitored_nodes)
        running_nodes = sum(1 for state in self.node_states.values() if state == 'running')
        error_nodes = sum(1 for state in self.node_states.values() if state in ['ERROR', 'TIMEOUT', 'FAILED'])

        if error_nodes == 0:
            overall_status = 'OK'
        elif error_nodes < total_nodes // 2:
            overall_status = 'WARNING'
        else:
            overall_status = 'ERROR'
        
        system_status = {
            'timestamp': datetime.now().isoformat(),
            'overall_status': overall_status,
            'total_nodes': total_nodes,
            'running_nodes': running_nodes,
            'error_nodes': error_nodes,
            'node_states': self.node_states,
            'node_health_data': self.node_health_data
        }
        
        status_msg = String()
        status_msg.data = json.dumps(system_status)
        self.system_status_pub.publish(status_msg)
        
        # Publikacja alertów jeśli są problemy
        if error_nodes > 0:
            alert_msg = String()
            alert_data = {
                'timestamp': datetime.now().isoformat(),
                'level': 'WARNING' if overall_status == 'WARNING' else 'ERROR',
                'message': f"System MSS: {error_nodes} węzłów ma problemy",
                'details': {name: state for name, state in self.node_states.items() if state != 'running'}
            }
            alert_msg.data = json.dumps(alert_data)
            self.health_alerts_pub.publish(alert_msg)
        
        # Publikacja szczegółowego statusu węzłów
        node_status_msg = String()
        node_status_data = {
            'timestamp': datetime.now().isoformat(),
            'nodes': {}
        }
        
        for node_name in self.monitored_nodes:
            node_status_data['nodes'][node_name] = {
                'status': self.node_states[node_name],
                'last_update': self.last_health_updates[node_name],
                'health_data': self.node_health_data[node_name]
            }
        
        node_status_msg.data = json.dumps(node_status_data)
        self.node_status_pub.publish(node_status_msg)

    def publish_health(self):
        """Publikuje status zdrowia węzła health monitor."""
        try:
            import psutil
            
            # Sprawdź status timerów
            status_timer_status = "OK" if hasattr(self, 'status_timer') else "ERROR"
            health_timer_status = "OK" if hasattr(self, 'health_timer') else "ERROR"
            
            # Sprawdź status publisherów
            system_status_pub_status = "OK" if hasattr(self, 'system_status_pub') else "ERROR"
            health_alerts_pub_status = "OK" if hasattr(self, 'health_alerts_pub') else "ERROR"
            
            # Sprawdź status subskrypcji
            subscription_status = "OK" if len(self.monitored_nodes) > 0 else "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if status_timer_status == "ERROR":
                errors.append("Timer status nieaktywny")
            if health_timer_status == "ERROR":
                errors.append("Timer health nieaktywny")
            if system_status_pub_status == "ERROR":
                errors.append("Publisher system status nieaktywny")
            if health_alerts_pub_status == "ERROR":
                errors.append("Publisher health alerts nieaktywny")
            if subscription_status == "ERROR":
                errors.append("Brak subskrypcji")
            
            # Sprawdź liczbę monitorowanych węzłów
            monitored_count = len(self.monitored_nodes)
            if monitored_count == 0:
                errors.append("Brak monitorowanych węzłów")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'status_timer_status': status_timer_status,
                'health_timer_status': health_timer_status,
                'system_status_pub_status': system_status_pub_status,
                'health_alerts_pub_status': health_alerts_pub_status,
                'subscription_status': subscription_status,
                'monitored_nodes_count': monitored_count,
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
    health_monitor_node = MSSHealthMonitorNode()
    
    try:
        rclpy.spin(health_monitor_node)
    except KeyboardInterrupt:
        health_monitor_node.get_logger().info("Health Monitor MSS zatrzymywany...")
    finally:
        health_monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
