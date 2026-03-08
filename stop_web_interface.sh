#!/bin/bash

# MSS Web Interface - Skrypt zatrzymywania
# Zatrzymuje system MSS przez systemd

echo "🛑 Zatrzymywanie MSS Web Interface..."
echo "===================================="

# Sprawdź czy usługa istnieje
if ! systemctl --user list-unit-files | grep -q "mss-web-interface.service"; then
    echo "❌ BŁĄD: Usługa mss-web-interface.service nie istnieje!"
    echo "   Sprawdź czy plik ~/.config/systemd/user/mss-web-interface.service istnieje"
    exit 1
fi

# Sprawdź status
if ! systemctl --user is-active --quiet mss-web-interface.service; then
    echo "⚠️  MSS Web Interface już jest zatrzymany!"
    echo "   Status: $(systemctl --user is-active mss-web-interface.service)"
    echo "   Aby uruchomić: ./start_web_interface.sh"
    exit 0
fi

# Zatrzymaj usługę
echo "🔄 Zatrzymywanie usługi..."
systemctl --user stop mss-web-interface.service

# Sprawdź czy się zatrzymała
sleep 3
if ! systemctl --user is-active --quiet mss-web-interface.service; then
    echo "✅ MSS Web Interface zatrzymany pomyślnie!"
    echo ""
    echo "💡 Aby uruchomić: ./start_web_interface.sh"
    echo "💡 Aby zrestartować: ./restart_web_interface.sh"
else
    echo "⚠️  Usługa nadal działa, próbuję force stop..."
    systemctl --user kill mss-web-interface.service
    sleep 2
    
    if ! systemctl --user is-active --quiet mss-web-interface.service; then
        echo "✅ MSS Web Interface zatrzymany (force stop)!"
    else
        echo "❌ BŁĄD: Nie udało się zatrzymać MSS Web Interface!"
        echo ""
        echo "📝 Sprawdź logi:"
        journalctl --user -u mss-web-interface.service -n 10 --no-pager
        echo ""
        echo "🔧 Spróbuj ręcznie:"
        echo "   systemctl --user stop mss-web-interface.service"
        echo "   systemctl --user kill mss-web-interface.service"
        exit 1
    fi
fi
