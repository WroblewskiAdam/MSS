#!/bin/bash

# MSS Startup Script
# Automatyczne uruchamianie systemu MSS

echo "=== MSS Startup Script ==="
echo "Data: $(date)"
echo "=========================="

# Sprawdzanie czy jesteśmy w odpowiednim katalogu
if [ ! -f "mss_bringup/launch/all_nodes.launch.py" ]; then
    echo "BŁĄD: Nie jestem w katalogu src projektu MSS!"
    echo "Przejdź do: cd /home/pi/mss_ros/src"
    exit 1
fi

# Sprawdzanie dostępności ROS2
if ! command -v ros2 &> /dev/null; then
    echo "BŁĄD: ROS2 nie jest zainstalowany lub nie jest w PATH!"
    exit 1
fi

# Sprawdzanie sprzętu (podstawowe testy)
echo "Sprawdzanie sprzętu..."

# Test GPIO
if [ ! -d "/sys/class/gpio" ]; then
    echo "OSTRZEŻENIE: GPIO nie jest dostępne"
else
    echo "✓ GPIO dostępne"
fi

# Test portów szeregowych
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "OSTRZEŻENIE: Port /dev/ttyUSB0 nie jest dostępny"
else
    echo "✓ Port szeregowy /dev/ttyUSB0 dostępny"
fi

# Sprawdzanie połączenia sieciowego
if ping -c 1 8.8.8.8 &> /dev/null; then
    echo "✓ Połączenie internetowe OK"
else
    echo "OSTRZEŻENIE: Brak połączenia internetowego"
fi

echo ""
            echo "Uruchamianie systemu MSS..."

            # Uruchomienie systemu MSS z health monitor
            cd /home/pi/mss_ros/src
            source /opt/ros/humble/setup.bash
            echo "Uruchamiam system MSS z health monitor..."
            ros2 launch mss_bringup all_nodes.launch.py

echo "System MSS zakończył działanie."
