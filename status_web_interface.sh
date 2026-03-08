#!/bin/bash

# MSS Web Interface - Skrypt sprawdzania statusu
# Pokazuje status systemu MSS

echo "📊 Status MSS Web Interface"
echo "=========================="

# Sprawdź czy usługa istnieje
if ! systemctl --user list-unit-files | grep -q "mss-web-interface.service"; then
    echo "❌ BŁĄD: Usługa mss-web-interface.service nie istnieje!"
    echo "   Sprawdź czy plik ~/.config/systemd/user/mss-web-interface.service istnieje"
    exit 1
fi

# Status usługi
echo "🔧 Status usługi:"
systemctl --user status mss-web-interface.service --no-pager -l

echo ""
echo "🌐 Sprawdzanie portów:"

# Sprawdź port 8080 (web interface)
if lsof -i :8080 >/dev/null 2>&1; then
    echo "✅ Port 8080 (Web Interface): DZIAŁA"
    echo "   http://localhost:8080"
else
    echo "❌ Port 8080 (Web Interface): NIE DZIAŁA"
fi

# Sprawdź port 9090 (ROS Bridge)
if lsof -i :9090 >/dev/null 2>&1; then
    echo "✅ Port 9090 (ROS Bridge): DZIAŁA"
    echo "   ws://localhost:9090"
else
    echo "❌ Port 9090 (ROS Bridge): NIE DZIAŁA"
fi

echo ""
echo "📝 Ostatnie logi (10 linii):"
journalctl --user -u mss-web-interface.service -n 10 --no-pager

echo ""
echo "💡 Komendy:"
echo "   ./start_web_interface.sh    - Uruchom"
echo "   ./stop_web_interface.sh     - Zatrzymaj"
echo "   ./restart_web_interface.sh  - Restart"
echo "   ./status_web_interface.sh   - Status (ten skrypt)"
