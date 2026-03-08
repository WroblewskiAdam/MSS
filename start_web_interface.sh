#!/bin/bash

# MSS Web Interface - Skrypt uruchamiania
# Uruchamia system MSS przez systemd

echo "🚀 Uruchamianie MSS Web Interface..."
echo "=================================="

# Sprawdź czy usługa istnieje
if ! systemctl --user list-unit-files | grep -q "mss-web-interface.service"; then
    echo "❌ BŁĄD: Usługa mss-web-interface.service nie istnieje!"
    echo "   Sprawdź czy plik ~/.config/systemd/user/mss-web-interface.service istnieje"
    exit 1
fi

# Sprawdź status
if systemctl --user is-active --quiet mss-web-interface.service; then
    echo "⚠️  MSS Web Interface już działa!"
    echo "   Status: $(systemctl --user is-active mss-web-interface.service)"
    echo "   Aby zrestartować: ./restart_web_interface.sh"
    echo "   Aby zatrzymać: ./stop_web_interface.sh"
    exit 0
fi

# Uruchom usługę
echo "🔄 Uruchamianie usługi..."
systemctl --user start mss-web-interface.service

# Sprawdź czy się uruchomiła
sleep 3
if systemctl --user is-active --quiet mss-web-interface.service; then
    echo "✅ MSS Web Interface uruchomiony pomyślnie!"
    echo ""
    echo "📱 Web Interface: http://localhost:8080"
    echo "🔌 ROS Bridge: ws://localhost:9090"
    echo ""
    echo "📊 Status:"
    systemctl --user status mss-web-interface.service --no-pager -l
    echo ""
    echo "💡 Aby zatrzymać: ./stop_web_interface.sh"
    echo "💡 Aby zrestartować: ./restart_web_interface.sh"
    echo "💡 Aby zobaczyć logi: journalctl --user -u mss-web-interface.service -f"
else
    echo "❌ BŁĄD: Nie udało się uruchomić MSS Web Interface!"
    echo ""
    echo "📝 Sprawdź logi:"
    journalctl --user -u mss-web-interface.service -n 20 --no-pager
    exit 1
fi
