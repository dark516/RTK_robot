#!/usr/bin/env python3
import socket
import threading
import time
from typing import Optional, Callable, Dict, Tuple

class ESPDevice:
    """Состояние и управление одним ESP32."""
    def __init__(self, hostname: str, base_index: int, port: int):
        self.hostname = hostname
        self.base_index = base_index   # 0 для датчиков 1,2; 2 для 3,4
        self.port = port
        self.ip: Optional[str] = None
        self.state = "DISCONNECTED"    # DISCONNECTED, RESOLVING, CALIBRATING, READY
        self.last_activity = 0.0
        self.calibration_sent = False
        self.calibration_ack = False

    def update_activity(self):
        self.last_activity = time.time()

class StableDualESPDetector:
    def __init__(self,
                 hostname1: str = "bunderkrivitka-12.local",
                 hostname2: str = "bunderkrivitka-34.local",
                 port: int = 8888,
                 on_find: Optional[Callable[[int], None]] = None,
                 calibration_timeout: float = 10.0,
                 rediscovery_interval: float = 5.0):
        self.hostname1 = hostname1
        self.hostname2 = hostname2
        self.port = port
        self.on_find = on_find
        self.calibration_timeout = calibration_timeout
        self.rediscovery_interval = rediscovery_interval

        self.devices = [
            ESPDevice(hostname1, 0, port),
            ESPDevice(hostname2, 2, port)
        ]
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def _resolve_hostname(self, dev: ESPDevice) -> bool:
        """Пытается разрешить hostname в IP. Возвращает True при успехе."""
        try:
            ip = socket.gethostbyname(dev.hostname)
            if dev.ip != ip:
                dev.ip = ip
                print(f"[{dev.hostname}] разрешён как {ip}")
                dev.update_activity()
            return True
        except socket.gaierror:
            return False

    def _send_calibration(self, dev: ESPDevice):
        """Отправляет команду калибровки (байт 0x01) на устройство."""
        if dev.ip is None:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(b'\x01', (dev.ip, self.port))
            sock.close()
            print(f"[{dev.hostname}] отправлена команда калибровки")
            dev.calibration_sent = True
            dev.update_activity()
        except Exception as e:
            print(f"[{dev.hostname}] ошибка отправки калибровки: {e}")

    def _process_packet(self, data: bytes, addr: Tuple[str, int]):
        """Обрабатывает входящий UDP пакет."""
        ip = addr[0]
        with self._lock:
            for dev in self.devices:
                if dev.ip == ip:
                    if len(data) == 1:
                        byte = data[0]
                        if byte == 0:
                            # Подтверждение калибровки
                            if dev.state == "CALIBRATING":
                                dev.state = "READY"
                                dev.calibration_ack = True
                                print(f"[{dev.hostname}] калибровка завершена, режим READY")
                        elif 1 <= byte <= 4:
                            # Индекс датчика (1..4) – преобразуем в 0..3
                            sensor_idx = byte - 1
                            if self.on_find:
                                self.on_find(sensor_idx)
                        dev.update_activity()
                    break

    def _udp_listener(self):
        """Фоновый поток для приёма UDP пакетов."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', self.port))
        self._sock.settimeout(0.5)
        print(f"UDP слушатель запущен на порту {self.port}")
        while self._running:
            try:
                data, addr = self._sock.recvfrom(1024)
                self._process_packet(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Ошибка приёма UDP: {e}")
        if self._sock:
            self._sock.close()

    def _maintenance_loop(self):
        """Периодически проверяет состояние устройств и управляет подключением/калибровкой."""
        while self._running:
            now = time.time()
            with self._lock:
                for dev in self.devices:
                    # Если устройство не в READY и прошло время rediscovery_interval с последней активности
                    if dev.state != "READY":
                        # Попытка разрешить имя, если IP отсутствует
                        if dev.ip is None:
                            if self._resolve_hostname(dev):
                                dev.state = "RESOLVING"
                                dev.update_activity()
                        # Если IP есть и мы не в процессе калибровки
                        if dev.ip is not None and dev.state != "CALIBRATING":
                            # Инициируем калибровку
                            dev.state = "CALIBRATING"
                            dev.calibration_sent = False
                            dev.calibration_ack = False
                            self._send_calibration(dev)
                        # Если в калибровке, но прошло больше calibration_timeout без подтверждения
                        if dev.state == "CALIBRATING" and dev.calibration_sent and (now - dev.last_activity > self.calibration_timeout):
                            print(f"[{dev.hostname}] таймаут калибровки, повторная попытка")
                            self._send_calibration(dev)
            time.sleep(self.rediscovery_interval)

    def start(self):
        """Запускает детектор (неблокирующий)."""
        if self._running:
            return
        self._running = True
        # Запускаем поток приёма UDP
        threading.Thread(target=self._udp_listener, daemon=True).start()
        # Запускаем поток обслуживания состояний
        threading.Thread(target=self._maintenance_loop, daemon=True).start()
        print("Детектор запущен. Ожидание появления ESP32...")

    def stop(self):
        """Останавливает детектор."""
        self._running = False

# ========== ПРИМЕР ИСПОЛЬЗОВАНИЯ ==========
def on_find(index: int):
    print(f"🔴 МАГНИТ НАЙДЕН НА ДАТЧИКЕ {index+1}!")

if __name__ == "__main__":
    detector = StableDualESPDetector(
        hostname1="bunderkrivitka-12.local",
        hostname2="bunderkrivitka-34.local",
        port=8888,
        on_find=on_find,
        calibration_timeout=10.0,
        rediscovery_interval=5.0
    )
    detector.start()
    print("Детектор работает. Нажмите Ctrl+C для выхода.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        detector.stop()
        print("\nДетектор остановлен.")