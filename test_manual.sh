#!/bin/bash
# Szybki start testów motor_driver_speed bez skryptów Pythona

echo "=================================================="
echo "TEST MOTOR_DRIVER_SPEED - WERSJA MANUALNA"
echo "=================================================="
echo ""
echo "Upewnij się że w innym terminalu jest uruchomiony:"
echo "  ros2 run mks_motor_control motor_driver_speed"
echo ""
echo "Menu testów:"
echo "  1) Sprawdź czy robot odbiera komendy"
echo "  2) Test jazdy na wprost (5 sekund, 0.5 m/s)"
echo "  3) Test jazdy po okręgu (20 sekund, radius=1m)"
echo ""
read -p "Wybierz test (1/2/3): " choice

case $choice in
    1)
        echo ""
        echo "TEST: Monitorowanie cmd_vel i odom"
        echo "Wysyłam testową komendę..."
        (sleep 1 && ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}") &
        
        echo ""
        echo "Monitoruję /cmd_vel i /odom..."
        ros2 topic echo /cmd_vel &
        CMD_PID=$!
        ros2 topic echo /odom &
        ODOM_PID=$!
        
        sleep 5
        kill $CMD_PID $ODOM_PID 2>/dev/null
        ;;
    
    2)
        echo ""
        echo "TEST 1: JAZDA NA WPROST"
        echo "Kierunek: do przodu"
        echo "Prędkość: 0.5 m/s"
        echo "Czas: 5 sekund"
        echo "Oczekiwanie droga: ~2.5 metra"
        echo ""
        read -p "Gotowy? (Enter aby kontynuować)"
        
        echo "Wysyłam komendę START..."
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" &
        PUB_PID=$!
        
        echo "Monitoruję odometrię przez 5 sekund..."
        echo ""
        
        START_TIME=$(date +%s)
        while [ $(($(date +%s) - START_TIME)) -lt 5 ]; do
            ros2 topic echo /odom -n 1 2>/dev/null | grep -E "position|orientation" | head -3
            sleep 1
        done
        
        kill $PUB_PID 2>/dev/null
        
        echo ""
        echo "Wysyłam komendę STOP..."
        ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" &
        sleep 1
        
        echo ""
        echo "Finalna pozycja robota:"
        ros2 topic echo /odom -n 1 2>/dev/null | grep -E "position|orientation"
        ;;
    
    3)
        echo ""
        echo "TEST 2: JAZDA PO OKRĘGU"
        echo "Promień: 1 metr"
        echo "Oczekiwana droga: ~6.28 metra (obwód)"
        echo "Oczekiwana rotacja: ~2π radianów (~360°)"
        echo "Czas: 20 sekund"
        echo ""
        read -p "Gotowy? (Enter aby kontynuować)"
        
        echo "Wysyłam komendę START..."
        echo "  linear.x = 0.314 m/s"
        echo "  angular.z = 0.314 rad/s"
        echo ""
        
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.314}, angular: {z: 0.314}}" &
        PUB_PID=$!
        
        echo "Monitoruję odometrię przez 20 sekund..."
        echo ""
        
        START_TIME=$(date +%s)
        while [ $(($(date +%s) - START_TIME)) -lt 20 ]; do
            ros2 topic echo /odom -n 1 2>/dev/null | grep -E "position|orientation" | head -3
            sleep 2
        done
        
        kill $PUB_PID 2>/dev/null
        
        echo ""
        echo "Wysyłam komendę STOP..."
        ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" &
        sleep 1
        
        echo ""
        echo "Finalna pozycja robota (powinny być blisko 0):"
        ros2 topic echo /odom -n 1 2>/dev/null | grep -E "position|orientation"
        ;;
    
    *)
        echo "Nieznana opcja!"
        ;;
esac

echo ""
echo "Test zakończony!"
