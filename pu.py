import serial
import logging
import time
from typing import Optional

PORT = "COM4"
BAUD = 9600
TIMEOUT_S = 2.0
UNITS_MM = 1e-4  # Единица ответа команды WHERE - это 0.1 микрона, поэтому множитель 1e-4 для перевода в мм. Он остается верным.

class SimpleMS2000:
    def __init__(self, port: str, baud: int, timeout: float):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.logger = logging.getLogger("SimpleMS2000")
        self.logger.setLevel(logging.INFO)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            self.logger.info(f"Подключён к {self.port}")
            # Сразу после подключения переключаемся в режим высокой точности
            time.sleep(0.2) # Даем порту "проснуться"
            self.set_high_precision()
        except Exception as e:
            self.logger.error(f"Не удалось открыть порт: {e}")
            self.ser = None

    def set_high_precision(self):
        """
        Переключает контроллер в режим высокой точности для команды WHERE.
        Отправляет команду 255 72 (Alt+255, H).
        """
        if not self.ser:
            return
        
        try:
            cmd = bytes([255, 72])
            self.logger.info("Отправка команды для переключения в режим высокой точности (255, 72).")
            self._flush()
            self.ser.write(cmd)
            # Эта команда не возвращает стандартного ответа :A, поэтому просто делаем небольшую паузу.
            time.sleep(0.1)
        except Exception as e:
            self.logger.error(f"Ошибка при установке режима высокой точности: {e}")

    def _flush(self) -> None:
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def _write(self, cmd: bytes, term: bytes = b'\r') -> Optional[str]:
        """Отправить команду и вернуть строку‑ответ без терминатора."""
        if not self.ser:
            return None
        try:
            # Флашить буферы лучше прямо перед записью
            self._flush()
            self.ser.write(cmd + term)
            raw = self.ser.read_until(b'\r\n')
            return raw.decode('utf-8').rstrip('\r\n')
        except Exception as e:
            self.logger.error(f"_write error: {e}")
            return None

    def get_position(self):
        """Запрос текущих координат (команда «W X Y»)."""
        resp = self._write(b'W X Y')
        if resp and resp.startswith(":A"):
            # В режиме низкой точности ответ: ":A 67977 12345"
            # В режиме высокой точности ответ: ":A 67977.2 12345.0"
            # Функция float() обработает оба варианта корректно.
            parts = resp.split()
            if len(parts) >= 3:
                x = float(parts[1]) * UNITS_MM # e.g. 67977.2 * 1e-4 = 6.79772
                y = float(parts[2]) * UNITS_MM
                self.logger.info(f"Позиция – X: {x:.6f} mm, Y: {y:.6f} mm")
                return x, y
        self.logger.warning(f"Неожиданный ответ на get_position(): {resp}")
        return None

    def get_speed(self):
        """Запрос текущих скоростей (команда «s x? y?`)."""
        resp = self._write(b's x? y?')
        if resp and resp.startswith(":A"):
            # пример: ":A X=5.0 Y=5.0"
            parts = resp.split()
            if len(parts) >= 3:
                sx = float(parts[1].split('=')[1])
                sy = float(parts[2].split('=')[1])
                self.logger.info(f"Скорость – X: {sx:.3f} mm/s, Y: {sy:.3f} mm/s")
                return sx, sy
        self.logger.warning(f"Неожиданный ответ на get_speed(): {resp}")
        return None

    def close(self):
        if self.ser:
            self.ser.close()
            self.logger.info("Порт закрыт")

def try_commands(dev: SimpleMS2000) -> None:
    """Пробует несколько вариантов запросов позиции и выводит ответы."""
    print("--- Проверка ответа команды W X Y ---")
    resp = dev._write(b'W X Y')
    print(f"  W X Y → {resp}")

if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s %(levelname)s %(message)s",
                        level=logging.INFO)

    dev = SimpleMS2000(PORT, BAUD, TIMEOUT_S)

    if dev.ser:
        try_commands(dev)
        dev.get_position()
        dev.get_speed()
        dev.close()