#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import time
import threading
from typing import Optional

import paho.mqtt.client as mqtt
from std_msgs.msg import String
from my_robot_interfaces.msg import GpsRtk
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class MQTTChopperReceiverNode(Node):
    """
    Węzeł do odbioru danych GPS z sieczkarni przez MQTT.
    """
    
    def __init__(self):
        super().__init__('mqtt_chopper_receiver_node')
        
        # --- Parametry MQTT ---
        self.declare_parameter('mqtt_broker_host', 'mss-mqtt.ddns.net')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_topic_chopper', 'test/polaczenia')
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Pobierz parametry
        self.mqtt_host = self.get_parameter('mqtt_broker_host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_broker_port').get_parameter_value().integer_value
        self.mqtt_topic = self.get_parameter('mqtt_topic_chopper').get_parameter_value().string_value
        self.mqtt_keepalive = self.get_parameter('mqtt_keepalive').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        
        # --- QoS Profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # --- Publisher ---
        self.gps_publisher = self.create_publisher(
            GpsRtk, 
            '/gps_rtk_data/chopper', 
            qos_profile
        )
        
        # --- Health Publisher ---
        self.health_publisher = self.create_publisher(
            String, 
            '/mss/node_health/mqtt_chopper_receiver', 
            qos_profile
        )
        
        # --- Zmienne stanu ---
        self.mqtt_client = None
        self.is_connected = False
        self.last_message_time = 0.0
        self.message_count = 0
        self.error_count = 0
        self.reconnect_timer = None
        
        # --- Inicjalizacja MQTT ---
        self.setup_mqtt_client()
        
        # --- Timery ---
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        self.get_logger().info(f"MQTT Chopper Receiver zainicjalizowany")
        self.get_logger().info(f"Broker: {self.mqtt_host}:{self.mqtt_port}")
        self.get_logger().info(f"Topic: {self.mqtt_topic}")
    
    def setup_mqtt_client(self):
        """Konfiguracja klienta MQTT"""
        try:
            self.mqtt_client = mqtt.Client()
            
            # Ustawienia uwierzytelniania
            if self.mqtt_username and self.mqtt_password:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
            
            # Callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_log = self.on_mqtt_log
            
            # Połącz z brokerem
            self.connect_to_broker()
            
        except Exception as e:
            self.get_logger().error(f"Błąd konfiguracji MQTT: {e}")
            self.error_count += 1
    
    def connect_to_broker(self):
        """Połączenie z brokerem MQTT"""
        try:
            self.get_logger().info(f"Łączenie z brokerem MQTT: {self.mqtt_host}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, self.mqtt_keepalive)
            
            # Uruchom pętlę MQTT w osobnym wątku
            self.mqtt_client.loop_start()
            
        except Exception as e:
            self.get_logger().error(f"Błąd połączenia z brokerem MQTT: {e}")
            self.error_count += 1
            self.schedule_reconnect()
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback połączenia MQTT"""
        if rc == 0:
            self.is_connected = True
            self.get_logger().info("Połączono z brokerem MQTT")
            
            # Subskrybuj topic
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f"Subskrybowano topic: {self.mqtt_topic}")
            
            # Anuluj timer reconnect jeśli istnieje
            if self.reconnect_timer:
                self.reconnect_timer.cancel()
                self.reconnect_timer = None
                
        else:
            self.is_connected = False
            self.get_logger().error(f"Błąd połączenia MQTT: {rc}")
            self.error_count += 1
            self.schedule_reconnect()
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback rozłączenia MQTT"""
        self.is_connected = False
        self.get_logger().warn(f"Rozłączono z brokerem MQTT: {rc}")
        self.schedule_reconnect()
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback otrzymania wiadomości MQTT"""
        try:
            # Dekoduj wiadomość JSON
            message_data = json.loads(msg.payload.decode('utf-8'))
            
            # Sprawdź czy wiadomość zawiera dane GPS
            if self.validate_gps_data(message_data):
                # Konwertuj na wiadomość ROS2
                gps_msg = self.convert_to_gps_rtk(message_data)
                
                # Opublikuj
                self.gps_publisher.publish(gps_msg)
                
                # Aktualizuj statystyki
                self.message_count += 1
                self.last_message_time = time.time()
                
                self.get_logger().debug(f"Otrzymano dane GPS z sieczkarni: {message_data}")
                
            else:
                self.get_logger().warn(f"Nieprawidłowe dane GPS: {message_data}")
                self.error_count += 1
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Błąd parsowania JSON: {e}")
            self.error_count += 1
        except Exception as e:
            self.get_logger().error(f"Błąd przetwarzania wiadomości: {e}")
            self.error_count += 1
    
    def on_mqtt_log(self, client, userdata, level, buf):
        """Callback logów MQTT"""
        if level == mqtt.MQTT_LOG_ERR:
            self.get_logger().error(f"MQTT Error: {buf}")
        elif level == mqtt.MQTT_LOG_WARNING:
            self.get_logger().warn(f"MQTT Warning: {buf}")
        else:
            self.get_logger().debug(f"MQTT: {buf}")
    
    def validate_gps_data(self, data):
        """Walidacja danych GPS - format sendera"""
        required_fields = ['lat', 'lon', 'speed', 'heading', 'rtk_status']
        return all(field in data for field in required_fields)
    
    def convert_to_gps_rtk(self, data):
        """Konwersja danych JSON na wiadomość GpsRtk - format sendera"""
        gps_msg = GpsRtk()
        
        # Ustaw timestamp
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'chopper_gps'
        
        # Dane GPS - format sendera
        gps_msg.latitude_deg = float(data['lat'])
        gps_msg.longitude_deg = float(data['lon'])
        gps_msg.altitude_m = float(data.get('altitude', 0.0))
        gps_msg.speed_mps = float(data['speed'])
        gps_msg.heading_deg = float(data['heading'])
        gps_msg.rtk_status = int(data['rtk_status'])
        
        return gps_msg
    
    def schedule_reconnect(self):
        """Zaplanuj ponowne połączenie"""
        if not self.reconnect_timer:
            self.reconnect_timer = self.create_timer(
                self.reconnect_interval, 
                self.reconnect_callback
            )
    
    def reconnect_callback(self):
        """Callback ponownego połączenia"""
        if not self.is_connected:
            self.get_logger().info("Próba ponownego połączenia z brokerem MQTT...")
            self.connect_to_broker()
    
    def publish_health(self):
        """Publikacja statusu zdrowia węzła"""
        try:
            health_data = {
                'status': 'running' if self.is_connected else 'error',
                'timestamp': time.time(),
                'mqtt_connected': self.is_connected,
                'broker_host': self.mqtt_host,
                'broker_port': self.mqtt_port,
                'topic': self.mqtt_topic,
                'message_count': self.message_count,
                'error_count': self.error_count,
                'last_message_time': self.last_message_time,
                'uptime_seconds': time.time() - self.get_clock().now().nanoseconds / 1e9
            }
            
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_publisher.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd publikacji health: {e}")
    
    def destroy_node(self):
        """Zamknij połączenie MQTT przy zamykaniu węzła"""
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mqtt_chopper_receiver_node = None
    try:
        mqtt_chopper_receiver_node = MQTTChopperReceiverNode()
        rclpy.spin(mqtt_chopper_receiver_node)
    except KeyboardInterrupt:
        pass
    finally:
        if mqtt_chopper_receiver_node:
            mqtt_chopper_receiver_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
