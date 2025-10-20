import serial
import time
from typing import Optional

# --- НАСТРОЙКИ ---
COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
MOVE_DISTANCE_X_MM = 0.1
PULSE_DURATION_MS = 10 
NUM_MOVES = 5

ser: Optional[serial.Serial] = None

def send_command(cmd: str, quiet: bool = False) -> str:
    """Отправляет команду и возвращает ответ. 'quiet' подавляет лог."""
    global ser
    if not quiet: print(f"  CMD > {cmd}")
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(f"{cmd}\r".encode('ascii'))
    response = ser.read_until(b'\r\n').decode('ascii').strip()
    if not quiet: print(f"  RSP < {response}")
    return response

def get_position() -> Optional[tuple[float, float]]:
    response = send_command("W X Y")
    if response and response.startswith(":A"):
        parts = response.split()
        return float(parts[1])/UNITS_MM_TO_DEVICE, float(parts[2])/UNITS_MM_TO_DEVICE
    return None

def wait_for_idle():
    """Ждет, пока столик не завершит движение, с таймаутом."""
    print("  Waiting for move to complete...")
    start_time = time.time()
    while time.time() - start_time < 10: # Таймаут 10 секунд
        if send_command("/", quiet=True) == 'N':
            print("  ...Move complete.")
            return
        time.sleep(0.05)
    raise TimeoutError("Stage did not become idle in 10 seconds!")

try:
    print(f"Connecting to {COM_PORT}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(0.2)
    ser.write(bytes([255, 72])); time.sleep(0.1); ser.read_all()
    print("Connection successful.")

    print("\n--- TTL SETUP ---")
    send_command(f"RT Y={PULSE_DURATION_MS}")
    send_command("TTL Y=2")

    print("\n--- TEST START ---")
    print("1. Getting initial position...")
    start_pos = get_position()
    if not start_pos: raise ConnectionError("Failed to get start position")
    start_x, start_y = start_pos
    print(f"   => Start Position: X = {start_x:.4f} mm, Y = {start_y:.4f} mm")
    
    for i in range(NUM_MOVES):
        print(f"\n--- Loop {i+1} of {NUM_MOVES} ---")
        target_x = start_x + MOVE_DISTANCE_X_MM * (i + 1)
        
        print(f"  Target: X={target_x:.4f}, Y={start_y:.4f}")
        send_command(f"M X={int(target_x * UNITS_MM_TO_DEVICE)} Y={int(start_y * UNITS_MM_TO_DEVICE)}")
       
        wait_for_idle()

        print("  >>> TTL PULSE SHOULD HAVE BEEN SENT! <<<")

        print("  Verifying position...")
        current_pos = get_position()
        if current_pos:
            print(f"  Current Position: X={current_pos[0]:.4f}, Y={current_pos[1]:.4f}")
        else:
            print("  Could not verify position after move!")
        
    print("\n--- FINAL POSITION CHECK ---")
    final_pos = get_position()
    if final_pos:
        print(f"   => Final Position: X = {final_pos[0]:.4f} mm, Y = {final_pos[1]:.4f} mm")

except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")

finally:
    if ser and ser.is_open:
        print("\n--- CLEANUP ---")
        send_command("TTL Y=0") 
        ser.close()
        print("Serial port closed.")