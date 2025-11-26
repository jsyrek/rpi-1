#!/usr/bin/env python3
"""
Skrypt testowy do walidacji motor_driver_speed.py

Test 1: Jazda na wprost przez 5 sekund
Test 2: Jazda po okręgu o promieniu 100cm
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class MotorDriverTester(Node):
    def __init__(self):
        super().__init__('motor_driver_tester')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.odom_data = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'vx': 0.0,
            'vz': 0.0
        }
        self.odom_history = []

        self.test_running = False
        self.test_start_time = None
        self.test_duration = 0
        self.test_name = ""

        self.get_logger().info("MotorDriverTester gotowy!")
        self.get_logger().info("Dostępne testy:")
        self.get_logger().info("  1. test_straight_line() - jazda na wprost")
        self.get_logger().info("  2. test_circle() - jazda po okręgu")

    def odom_callback(self, msg: Odometry):
        """Callback do aktualizacji danych odometrii"""
        self.odom_data['x'] = msg.pose.pose.position.x
        self.odom_data['y'] = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.odom_data['theta'] = 2 * math.atan2(z, w)
        self.odom_data['vx'] = msg.twist.twist.linear.x
        self.odom_data['vz'] = msg.twist.twist.angular.z

        if self.test_running:
            self.odom_history.append({
                'time': time.time() - self.test_start_time,
                **self.odom_data.copy()
            })

    def publish_cmd_vel(self, linear, angular):
        """Wyślij komendę prędkości"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"CMD: linear={linear:.3f} m/s, angular={angular:.3f} rad/s")

    def stop_robot(self):
        """Zatrzymaj robota"""
        self.publish_cmd_vel(0.0, 0.0)

    def test_straight_line(self, duration=5.0, velocity=0.5):
        """Test 1: Jazda na wprost"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 1: JAZDA NA WPROST")
        self.get_logger().info(f"Czas trwania: {duration}s, Prędkość: {velocity} m/s")
        self.get_logger().info("=" * 60)

        self.test_name = "Straight Line"
        self.test_running = True
        self.test_start_time = time.time()
        self.test_duration = duration
        self.odom_history = []

        self.publish_cmd_vel(velocity, 0.0)

        end_time = time.time() + duration
        # ❗ ZAMIANA: pętla z spin_once zamiast sleep!
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.test_running = False

        # odbierz ostatnie próbki
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        self._analyze_results(velocity, 0.0, duration)

    def test_circle(self, duration=20.0, radius=1.0):
        """Test 2: Jazda po okręgu"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 2: JAZDA PO OKRĘGU")
        self.get_logger().info(f"Promień: {radius}m, Czas obrotu: {duration}s")
        self.get_logger().info("=" * 60)

        circumference = 2 * math.pi * radius
        linear_vel = circumference / duration
        angular_vel = linear_vel / radius

        self.get_logger().info(f"Obliczenia:")
        self.get_logger().info(f"  Obwód: {circumference:.3f}m")
        self.get_logger().info(f"  Prędkość liniowa: {linear_vel:.3f} m/s")
        self.get_logger().info(f"  Prędkość kątowa: {angular_vel:.3f} rad/s")

        self.test_name = "Circle"
        self.test_running = True
        self.test_start_time = time.time()
        self.test_duration = duration
        self.odom_history = []

        self.publish_cmd_vel(linear_vel, angular_vel)

        end_time = time.time() + duration
        # ❗ ZAMIANA: pętla z spin_once zamiast sleep!
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.test_running = False

        # odbierz ostatnie próbki
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        self._analyze_results(linear_vel, angular_vel, duration)

    def _analyze_results(self, linear_vel, angular_vel, duration):
        """Analiza wyników testu"""
        if not self.odom_history:
            self.get_logger().error("Brak danych odometrii!")
            return

        start = self.odom_history[0]
        end = self.odom_history[-1]

        delta_x = end['x'] - start['x']
        delta_y = end['y'] - start['y']
        delta_theta = end['theta'] - start['theta']
        distance = math.sqrt(delta_x**2 + delta_y**2)

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"WYNIKI TESTU: {self.test_name}")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"\nPozycja początkowa: x={start['x']:.4f}, y={start['y']:.4f}, θ={start['theta']:.4f}")
        self.get_logger().info(f"Pozycja końcowa:    x={end['x']:.4f}, y={end['y']:.4f}, θ={end['theta']:.4f}")
        self.get_logger().info(f"\nDelta X:      {delta_x:.4f}m")
        self.get_logger().info(f"Delta Y:      {delta_y:.4f}m")
        self.get_logger().info(f"Delta Theta:  {delta_theta:.4f} rad ({math.degrees(delta_theta):.2f}°)")
        self.get_logger().info(f"Droga (euclidean): {distance:.4f}m")

        self.get_logger().info("\n" + "-" * 60)
        self.get_logger().info("WALIDACJA:")
        self.get_logger().info("-" * 60)

        if self.test_name == "Straight Line":
            expected_distance = linear_vel * duration
            error_percent = abs(delta_x - expected_distance) / expected_distance * 100

            self.get_logger().info(f"Oczekiwana droga: {expected_distance:.4f}m")
            self.get_logger().info(f"Rzeczywista droga (x): {delta_x:.4f}m")
            self.get_logger().info(f"Błąd: {error_percent:.2f}%")

            if abs(delta_y) > 0.05:
                self.get_logger().warn(f"⚠️ UWAGA: Odboczenie w osi Y = {delta_y:.4f}m (powinno być ~0)")
            else:
                self.get_logger().info(f"✅ Jazda na wprost OK (Y offset = {delta_y:.4f}m)")

            if abs(delta_theta) > 0.1:
                self.get_logger().warn(f"⚠️ UWAGA: Zmiana orientacji = {delta_theta:.4f} rad (powinno być ~0)")
            else:
                self.get_logger().info(f"✅ Orientacja stabilna (θ offset = {delta_theta:.4f} rad)")

        elif self.test_name == "Circle":
            expected_circumference = 2 * math.pi * 1.0  # Dla promienia 1m
            error_percent = abs(distance - expected_circumference) / expected_circumference * 100

            self.get_logger().info(f"Oczekiwana droga (obwód 1m): {expected_circumference:.4f}m")
            self.get_logger().info(f"Rzeczywista droga: {distance:.4f}m")
            self.get_logger().info(f"Błąd: {error_percent:.2f}%")

            if abs(delta_theta) < math.pi * 1.5:  # Powinno być ~2π
                self.get_logger().warn(f"⚠️ UWAGA: Rotacja niedostateczna = {delta_theta:.4f} rad (oczekiwane ~{2*math.pi:.4f})")
            else:
                self.get_logger().info(f"✅ Rotacja OK (θ zmiana = {delta_theta:.4f} rad)")

            closure_error = math.sqrt(delta_x**2 + delta_y**2)
            self.get_logger().info(f"Błąd zamknięcia okręgu: {closure_error:.4f}m")
            if closure_error < 0.1:
                self.get_logger().info(f"✅ Okrąg zamknięty poprawnie!")
            else:
                self.get_logger().warn(f"⚠️ Okrąg niedostatecznie zamknięty!")

        self.get_logger().info("\n" + "=" * 60)

def main(args=None):
    rclpy.init(args=args)
    tester = MotorDriverTester()

    try:
        print("\nCzekam na połączenie z odometrią...")
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=0.5)

        print("\n" + "=" * 60)
        print("TESTY MOTOR_DRIVER_SPEED")
        print("=" * 60)

        print("\nRozpoczynanie TEST 1 za 2 sekundy...")
        time.sleep(2)
        tester.test_straight_line(duration=5.0, velocity=0.5)

        print("\nPrzerwa 3 sekundy...")
        time.sleep(3)

        print("\nRozpoczynanie TEST 2 za 2 sekundy...")
        time.sleep(2)
        tester.test_circle(duration=20.0, radius=1.0)

        print("\n" + "=" * 60)
        print("TESTY UKOŃCZONE!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\nTest przerwany przez użytkownika")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
