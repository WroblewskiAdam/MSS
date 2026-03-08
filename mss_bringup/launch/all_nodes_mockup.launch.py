# # mss_bringup/launch/all_nodes_mockup.launch.py
# Launch file dla systemu MSS z mockup danymi GPS

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    """Uruchamia wszystkie niezbędne węzły dla systemu MSS z mockup danymi GPS."""
    return LaunchDescription([
        
        # === MOCKUP: Węzeł symulujący dane GPS ===
        Node(
            package='system_mockup',
            executable='gps_mockup_node',
            name='gps_mockup_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'publish_frequency_hz': 10.0,
                'tractor_speed_kmh': 7.2,  # 7.2 km/h = ~2.0 m/s
                'chopper_speed_kmh': 7.2,  # 6.5 km/h = ~1.8 m/s (nieco wolniejsza)
                'chopper_offset_m': 7.0,   # 7m za ciągnikiem
                'simulation_area_lat': 50.0647,  # Kraków
                'simulation_area_lon': 19.9450,
            }]
        ),
        
        # === Węzły sensorów i komunikacji (bez prawdziwych sensorów) ===
        
        # Node(
        #     package='gps_rtk_reader',
        #     executable='gps_rtk_node',
        #     name='gps_rtk_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        # Node(
        #     package='bt_comm',
        #     executable='bt_receiver_node',
        #     name='bt_receiver_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        
        # === MOCKUP: Węzeł symulujący dane biegów ===
        Node(
            package='system_mockup',
            executable='gear_mockup_node',
            name='gear_mockup_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'publish_frequency_hz': 10.0,
                'shift_delay_sec': 1.0,
                'initial_gear': 1,
                'max_gear': 4,
                'min_gear': 1,
            }]
        ),

        # === Węzły wykonawcze (aktuatory) ===
        Node(
            package='servo_controller',
            executable='servo_node',
            name='servo_controller',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gear_controller',
            executable='gear_shifter',
            name='gear_shifter',
            output='screen',
            emulate_tty=True
        ),

        # === Węzły logiki i sterowania ===
        Node(
            package='mss_filters',
            executable='tractor_filter_node',
            name='tractor_filter_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='mss_filters',
            executable='chopper_filter_node',
            name='chopper_filter_node',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='speed_controller',
        #     executable='speed_controller_node',
        #     name='speed_controller_node',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        Node(
            package='position_controller',
            executable='position_controller_node',
            name='position_controller_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_distance': 0.0,
                'position_tolerance': 1.0,
                'speed_tolerance': 0.41,
                'Kp': 1.0,
                'Ki': 0.1,
                'min_speed': 0.5,
                'max_speed': 8.0,
                'max_acceleration': 0.5,
                'gps_timeout': 2.0,
                'control_frequency': 10.0
            }]
        ),
        # Node(
        #     package='relative_position_computer',
        #     executable='relative_computer_node',
        #     name='relative_computer_node',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        Node(
            package='gear_manager',
            executable='gear_manager_node',
            name='gear_manager_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # === Węzły pomocnicze (kluczowe dla UI) ===
        Node(
            package='mss_diagnostics',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # === Data Logger ===
        Node(
            package='data_logger',
            executable='mss_data_logger_node',
            name='mss_data_logger_node',
            output='screen',
            emulate_tty=True,
        ),
                    
        # === System Monitor RPi ===
        TimerAction(
            period=5.0,  # 5 sekund delay
            actions=[
                Node(
                    package='mss_system_monitor',
                    executable='system_monitor_node',
                    name='system_monitor_node',
                    output='screen',
                    emulate_tty=True,
                )
            ]
        ),
        
        # === Health Monitor ===
        TimerAction(
            period=10.0,  # 10 sekund delay
            actions=[
                Node(
                    package='mss_health_monitor',
                    executable='health_monitor_node',
                    name='mss_health_monitor_node',
                    output='screen',
                    emulate_tty=True,
                )
            ]
        ),
    ])
