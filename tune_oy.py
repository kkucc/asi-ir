import serial
import time
from typing import Optional

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
CALIBRATION_DISTANCE_MM = 15.0

ser: Optional[serial.Serial] = None

def send_command(cmd: str) -> str:
    """Универсальная функция для команд, возвращающих полную строку."""
    global ser
    print(f"  CMD > {cmd}")
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(f"{cmd}\r".encode('ascii'))
    response = ser.read_until(b'\r\n').decode('ascii').strip()
    print(f"  RSP < {response}")
    return response

def get_position() -> Optional[tuple[float, float]]:
    response = send_command("W X Y")
    if response and response.startswith(":A"):
        parts = response.split()
        return float(parts[1])/UNITS_MM_TO_DEVICE, float(parts[2])/UNITS_MM_TO_DEVICE
    return None

def wait_for_idle():
    """
    ИСПРАВЛЕНО: Специализированная функция, которая правильно читает
    односимвольный ответ от команды '/'.
    """
    print("  Waiting for move to complete (polling status with '/')...")
    start_time = time.time()
    while time.time() - start_time < 20: # Общий таймаут 20 секунд
        try:
            ser.reset_input_buffer()
            ser.write(b'/\r')
            response_byte = ser.read(1) # Читаем ровно 1 байт

            if not response_byte:
                print("  (timeout on read, retrying...)")
                continue
            
            status = response_byte.decode('ascii')
            print(f"  Got status: '{status}'") 
            if status == 'N':
                print("  ...Move complete (Got 'N').")
                return
            
            time.sleep(0.1) # Небольшая пауза между запросами
        except serial.SerialException as e:
            print(f"  Serial error while waiting: {e}")
            time.sleep(0.5)

    raise TimeoutError("Stage did not become idle!")

try:
    print(f"Connecting to {COM_PORT}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.5) # Уменьшил таймаут порта
    time.sleep(0.2); ser.write(bytes([255, 72])); time.sleep(0.1); ser.read_all()
    print("Connection successful.")

    print("\n--- Orthogonality Calibration ---")
    print("\n1. Moving to starting point (0, 0)...")
    send_command(f"M X=0 Y=0")
    wait_for_idle()
    
    start_pos = get_position()
    if not start_pos: raise ConnectionError("Failed to get start position")
    start_x, start_y = start_pos
    print(f"   => Confirmed Start: X={start_x:.6f}, Y={start_y:.6f}")

    print(f"\n2. Moving {CALIBRATION_DISTANCE_MM} mm along X-axis...")
    target_x = start_x + CALIBRATION_DISTANCE_MM
    send_command(f"M X={int(target_x * UNITS_MM_TO_DEVICE)} Y={int(start_y * UNITS_MM_TO_DEVICE)}")
    wait_for_idle()

    end_pos = get_position()
    if not end_pos: raise ConnectionError("Failed to get end position")
    end_x, end_y = end_pos
    print(f"   => Confirmed End:   X={end_x:.6f}, Y={end_y:.6f}")
    
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    
    if abs(delta_x) < 1.0:
        print("\nERROR: Stage did not move significantly along X. Cannot calculate.")
    else:
        correction_factor = delta_y / delta_x
        print("\n--- CALIBRATION COMPLETE ---")
        print(f"Y-axis drifted by {delta_y:.6f} mm over {delta_x:.2f} mm of X travel.")
        print("\nCOPY THIS VALUE INTO THE GUI:")
        print("===================================")
        print(f" Ortho. Correction: {correction_factor:.8f}")
        print("===================================")

except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")
finally:
    if ser and ser.is_open:
        ser.close()
        print("\nSerial port closed.")