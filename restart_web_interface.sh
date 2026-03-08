#!/bin/bash

# MSS Web Interface - Skrypt restartowania
# Restartuje system MSS przez systemd

echo "🔄 Restartowanie MSS Web Interface..."
echo "===================================="

# Sprawdź czy usługa istnieje
if ! systemctl --user list-unit-files | grep -q "mss-web-interface.service"; then
    echo "❌ BŁĄD: Usługa mss-web-interface.service nie istnieje!"
    echo "   Sprawdź czy plik ~/.config/systemd/user/mss-web-interface.service istnieje"
    exit 1
fi

# Zatrzymaj usługę
echo "🛑 Zatrzymywanie usługi..."
systemctl --user stop mss-web-interface.service

# Poczekaj na zatrzymanie
sleep 3

# Uruchom usługę
echo "🚀 Uruchamianie usługi..."
systemctl --user start mss-web-interface.service

# Sprawdź czy się uruchomiła
sleep 5
if systemctl --user is-active --quiet mss-web-interface.service; then
    echo "✅ MSS Web Interface zrestartowany pomyślnie!"
    echo ""
    echo "📱 Web Interface: http://localhost:8080"
    echo "🔌 ROS Bridge: ws://localhost:9090"
    echo ""
    echo "📊 Status:"
    systemctl --user status mss-web-interface.service --no-pager -l
    echo ""
    echo "💡 Aby zatrzymać: ./stop_web_interface.sh"
    echo "💡 Aby zobaczyć logi: journalctl --user -u mss-web-interface.service -f"
else
    echo "❌ BŁĄD: Nie udało się zrestartować MSS Web Interface!"
    echo ""
    echo "📝 Sprawdź logi:"
    journalctl --user -u mss-web-interface.service -n 20 --no-pager
    exit 1
fi
