#!/bin/bash

# MSS Web Interface Startup Script
# Automatyczne uruchamianie web interface z ROS Bridge
# Ctrl+C zatrzymuje wszystko

echo "=== MSS Web Interface Startup ==="
echo "Data: $(date)"
echo "================================"

# Sprawdzanie czy jesteśmy w odpowiednim katalogu
if [ ! -d "operator_interface/web" ]; then
    echo "BŁĄD: Nie jestem w katalogu src!"
    echo "Przejdź do: cd mss_ros/src"
    exit 1
fi

# Sprawdzanie dostępności ROS2
if ! command -v ros2 &> /dev/null; then
    echo "BŁĄD: ROS2 nie jest zainstalowany lub nie jest w PATH!"
    echo "Uruchom: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Sprawdzanie dostępności rosbridge_server
if ! ros2 pkg list | grep -q "rosbridge_server"; then
    echo "BŁĄD: rosbridge_server nie jest zainstalowany!"
    echo "Zainstaluj: sudo apt install ros-humble-rosbridge-server"
    exit 1
fi

# === AUTOMATYCZNE CZYSZCZENIE STARYCH SESJI ===

echo "🔍 Sprawdzanie starych sesji ROS Bridge..."

# Znajdź i zabij procesy rosbridge_server
ROSBRIDGE_PIDS=$(pgrep -f "rosbridge_server")
if [ ! -z "$ROSBRIDGE_PIDS" ]; then
    echo "⚠️  Znaleziono stare sesje ROS Bridge (PIDs: $ROSBRIDGE_PIDS)"
    echo "🗑️  Zabijam stare sesje..."
    
    for pid in $ROSBRIDGE_PIDS; do
        echo "   Zabijam proces $pid..."
        kill -TERM $pid 2>/dev/null
        sleep 1
        
        # Jeśli proces nadal żyje, użyj force kill
        if kill -0 $pid 2>/dev/null; then
            echo "   Force kill procesu $pid..."
            kill -KILL $pid 2>/dev/null
        fi
    done
    
    # Poczekaj chwilę na zamknięcie
    sleep 2
    
    # Sprawdź czy port 9090 jest wolny
    if lsof -i :9090 >/dev/null 2>&1; then
        echo "⚠️  Port 9090 nadal zajęty, próbuję force kill..."
        sudo lsof -ti :9090 | xargs -r sudo kill -KILL
        sleep 1
    fi
    
    echo "✅ Stare sesje ROS Bridge zostały zabite"
else
    echo "✅ Brak starych sesji ROS Bridge"
fi

# Sprawdź czy port 9090 jest wolny
if lsof -i :9090 >/dev/null 2>&1; then
    echo "❌ Port 9090 nadal zajęty! Sprawdź co go blokuje:"
    lsof -i :9090
    exit 1
fi

echo "✅ Port 9090 jest wolny"

# === URUCHAMIANIE ROS BRIDGE ===

echo "🚀 Uruchamianie ROS Bridge Server..."

# Uruchom rosbridge_server w tle
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!

# Poczekaj na uruchomienie
echo "⏳ Oczekiwanie na uruchomienie ROS Bridge..."
sleep 5

# Sprawdź czy ROS Bridge działa
if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
    echo "❌ ROS Bridge nie uruchomił się!"
    echo "Sprawdź logi: cat rosbridge.log"
    exit 1
fi

# Sprawdź czy port 9090 jest otwarty
if ! lsof -i :9090 >/dev/null 2>&1; then
    echo "❌ Port 9090 nie został otwarty!"
    echo "Sprawdź logi: cat rosbridge.log"
    kill $ROSBRIDGE_PID 2>/dev/null
    exit 1
fi

echo "✅ ROS Bridge uruchomiony (PID: $ROSBRIDGE_PID, Port: 9090)"

# === URUCHAMIANIE WEB SERVERA ===

echo "🌐 Uruchamianie web servera..."

# Sprawdź czy masz Python3
if command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null; then
    PYTHON_CMD="python"
else
    echo "❌ Nie znaleziono Pythona!"
    exit 1
fi

# Przejdź do katalogu web
cd operator_interface/web

# Uruchom web server w tle (z API dla logów)
$PYTHON_CMD server.py 8080 > webserver.log 2>&1 &
WEBSERVER_PID=$!

# Wróć do katalogu src
cd ../..

# Poczekaj na uruchomienie
sleep 2

# Sprawdź czy web server działa
if ! kill -0 $WEBSERVER_PID 2>/dev/null; then
    echo "❌ Web server nie uruchomił się!"
    echo "Sprawdź logi: cat operator_interface/web/webserver.log"
    kill $ROSBRIDGE_PID 2>/dev/null
    exit 1
fi

# Sprawdź czy port 8080 jest otwarty
if ! lsof -i :8080 >/dev/null 2>&1; then
    echo "❌ Port 8080 nie został otwarty!"
    echo "Sprawdź logi: cat operator_interface/web/webserver.log"
    kill $WEBSERVER_PID 2>/dev/null
    kill $ROSBRIDGE_PID 2>/dev/null
    exit 1
fi

echo "✅ Web server uruchomiony (PID: $WEBSERVER_PID, Port: 8080)"

# === INFORMACJE KOŃCOWE ===

echo ""
echo "🎉 MSS Web Interface uruchomiony pomyślnie!"
echo ""
echo "📱 Web Interface: http://localhost:8080"
echo "🔌 ROS Bridge: ws://localhost:9090"
echo ""
echo "📊 Status procesów:"
echo "   ROS Bridge: PID $ROSBRIDGE_PID (Port 9090)"
echo "   Web Server: PID $WEBSERVER_PID (Port 8080)"
echo ""
echo "📝 Logi:"
echo "   ROS Bridge: cat rosbridge.log"
echo "   Web Server: cat operator_interface/web/webserver.log"
echo ""
echo "💡 Zatrzymaj: Ctrl+C"
echo ""

# Zapisz PID-y do pliku dla łatwego zatrzymania
echo "$ROSBRIDGE_PID $WEBSERVER_PID" > .mss_pids
echo "PID-y zapisane w pliku .mss_pids"

# === FUNKCJA CLEANUP ===

cleanup() {
    echo ""
    echo "🛑 Zatrzymywanie MSS Web Interface..."
    
    if [ ! -z "$ROSBRIDGE_PID" ]; then
        kill $ROSBRIDGE_PID 2>/dev/null
        echo "   ROS Bridge zatrzymany"
    fi
    
    if [ ! -z "$WEBSERVER_PID" ]; then
        kill $WEBSERVER_PID 2>/dev/null
        echo "   Web server zatrzymany"
    fi
    
    # Usuń plik PID-ów
    rm -f .mss_pids
    
    # Sprawdź czy porty są wolne
    if lsof -i :9090 >/dev/null 2>&1; then
        echo "⚠️  Port 9090 nadal zajęty, force kill..."
        sudo lsof -ti :9090 | xargs -r sudo kill -KILL
    fi
    
    if lsof -i :8080 >/dev/null 2>&1; then
        echo "⚠️  Port 8080 nadal zajęty, force kill..."
        sudo lsof -ti :8080 | xargs -r sudo kill -KILL
    fi
    
    echo "✅ Wszystko zatrzymane"
    exit 0
}

# Przechwyć sygnały i wykonaj cleanup
trap cleanup SIGINT SIGTERM

# === GŁÓWNA PĘTLA ===

echo "💡 Naciśnij Ctrl+C aby zatrzymać"
echo ""

# Czekaj na sygnał zatrzymania
wait
