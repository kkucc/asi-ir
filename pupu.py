import serial
import time

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
MOVE_DISTANCE_X_MM = 5.0  

ser = None

def send_command(cmd: str) -> str:
    """Отправляет команду и возвращает ответ."""
    print(f"  CMD > {cmd}")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(f"{cmd}\r".encode('ascii'))
    response = ser.read_until(b'\r\n').decode('ascii').strip()
    print(f"  RSP < {response}")
    return response

def get_position() -> tuple[float, float] | None:
    """Запрашивает текущую позицию и возвращает ее в мм."""
    response = send_command("W X Y")
    if response and response.startswith(":A"):
        try:
            parts = response.split()
            x_mm = float(parts[1]) / UNITS_MM_TO_DEVICE
            y_mm = float(parts[2]) / UNITS_MM_TO_DEVICE
            return x_mm, y_mm
        except (ValueError, IndexError):
            print("  ERROR: Could not parse position response!")
            return None
    return None

def wait_for_idle():
    """Ждет, пока столик не завершит движение."""
    print("  Waiting for move to complete...")
    start_time = time.time()
    while time.time() - start_time < 10: # Таймаут 10 секунд
        if send_command("/") == 'N':
            print("  ...Move complete.")
            return
        time.sleep(0.05)
    raise TimeoutError("Stage did not become idle in time.")

try:
    print(f"Connecting to {COM_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(0.2) # Пауза для инициализации

    # Включаем режим высокой точности
    print("Setting high precision mode...")
    ser.write(bytes([255, 72]))
    time.sleep(0.1)
    ser.read_all() # Очищаем буфер от любого мусора

    print("\n--- TEST START ---")
    
    # 1. Получаем начальную позицию
    print("1. Getting initial position...")
    initial_pos = get_position()
    if initial_pos is None:
        raise ConnectionError("Failed to get initial position.")
    
    start_x, start_y = initial_pos
    print(f"   => Start Position: X = {start_x:.4f} mm, Y = {start_y:.4f} mm")
    
    # 2. Двигаем только ось X
    target_x = start_x + MOVE_DISTANCE_X_MM
    print(f"\n2. Moving X by {MOVE_DISTANCE_X_MM} mm, keeping Y constant...")
    send_command(f"M X={int(target_x * UNITS_MM_TO_DEVICE)} Y={int(start_y * UNITS_MM_TO_DEVICE)}")
    
    # 3. Ждем завершения движения
    wait_for_idle()

    # 4. Получаем конечную позицию
    print("\n3. Getting final position...")
    final_pos = get_position()
    if final_pos is None:
        raise ConnectionError("Failed to get final position.")
        
    end_x, end_y = final_pos
    print(f"   => Final Position: X = {end_x:.4f} mm, Y = {end_y:.4f} mm")

    # 5. Сравниваем и выносим вердикт
    print("\n--- ANALYSIS ---")
    delta_y = abs(start_y - end_y)
    print(f"Initial Y: {start_y:.4f} mm")
    print(f"Final Y:   {end_y:.4f} mm")
    print(f"Change in Y (delta): {delta_y:.6f} mm")

    if delta_y < 0.0001: # Допуск в 0.1 микрона на погрешность
        print("\nVERDICT: SUCCESS! The Y-axis did not move physically.")
        print("The problem is 100% a visual bug in the GUI rendering code.")
    else:
        print("\nVERDICT: FAILURE! The Y-axis IS physically moving.")
        print("The problem is in the controller or hardware.")

except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")

finally:
    if ser and ser.is_open:
        ser.close()
        print("\nSerial port closed.")