#!/bin/bash

# MSS Mockup Startup Script
# Automatyczne uruchamianie systemu MSS z mockup danymi GPS
# Ctrl+C zatrzymuje wszystko

echo "=== MSS Mockup Startup Script ==="
echo "Data: $(date)"
echo "================================"

# Sprawdzanie czy jesteśmy w odpowiednim katalogu
if [ ! -f "mss_bringup/launch/all_nodes_mockup.launch.py" ]; then
    echo "BŁĄD: Nie jestem w katalogu src projektu MSS!"
    echo "Przejdź do: cd /home/pi/mss_ros/src"
    exit 1
fi

# Sprawdzanie dostępności ROS2
if ! command -v ros2 &> /dev/null; then
    echo "BŁĄD: ROS2 nie jest zainstalowany lub nie jest w PATH!"
    echo "Uruchom: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Sprawdzanie czy pakiet system_mockup jest zbudowany
if [ ! -d "/home/pi/mss_ros/install/system_mockup" ]; then
    echo "BŁĄD: Pakiet system_mockup nie jest zbudowany!"
    echo "Uruchom: cd /home/pi/mss_ros && colcon build --packages-select system_mockup"
    exit 1
fi

# Sprawdzanie sprzętu (podstawowe testy)
echo "Sprawdzanie sprzętu..."

# Test GPIO
if [ ! -d "/sys/class/gpio" ]; then
    echo "OSTRZEŻENIE: GPIO nie jest dostępne (OK dla mockup)"
else
    echo "✓ GPIO dostępne"
fi

# Test portów szeregowych
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "OSTRZEŻENIE: Port /dev/ttyUSB0 nie jest dostępny (OK dla mockup)"
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
echo "🚀 Uruchamianie systemu MSS z mockup danymi GPS..."

# Uruchomienie systemu MSS z mockup danymi
cd /home/pi/mss_ros/src
source /opt/ros/humble/setup.bash
source /home/pi/mss_ros/install/setup.bash

echo "📡 Uruchamiam system MSS z mockup danymi GPS..."
echo "   - Ciągnik: symulowany GPS RTK"
echo "   - Sieczkarnia: symulowany GPS RTK (5m za ciągnikiem)"
echo "   - Prędkość: 2 m/s (~7.2 km/h)"
echo "   - Lokalizacja: Warszawa (52.2297, 21.0122)"
echo ""

ros2 launch mss_bringup all_nodes_mockup.launch.py

echo "System MSS z mockup danymi zakończył działanie."
