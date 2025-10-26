import serial
import time
from typing import Optional

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
MOVE_DISTANCE_X_MM = 0.5
DWELL_TIME_MS = 100     # (dwell)
NUM_MOVES = 5

ser: Optional[serial.Serial] = None

def send_command(cmd: str, quiet: bool = False) -> str:
    """Отправляет команду и возвращает ответ."""
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
    """Ждет, пока столик не завершит движение И аппаратное ожидание."""
    print("  Waiting for sequence (Move + Dwell) to complete...")
    start_time = time.time()
    while time.time() - start_time < 10:
        if send_command("/", quiet=True) == 'N':
            print("  ...Sequence complete.")
            return
        time.sleep(0.05)
    raise TimeoutError("Stage did not become idle in 10 seconds!")

def fire_ttl_pulse():
    """Вручную генерирует короткий TTL импульс."""
    print("  >>> Firing manual TTL pulse! <<<")
    send_command("TTL Y=1", quiet=True)
    send_command("TTL Y=0", quiet=True)

try:
    print(f"Connecting to {COM_PORT}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(0.2); ser.write(bytes([255, 72])); time.sleep(0.1); ser.read_all()
    print("Connection successful.")

    print("\n--- HARDWARE SETUP ---")
    print(f"1. Setting hardware WAIT (dwell) to {DWELL_TIME_MS} ms...")
    send_command(f"WT X={DWELL_TIME_MS} Y={DWELL_TIME_MS}")
    
    print("2. Setting TTL to manual mode (TTL Y=0) for software triggering...")
    send_command("TTL Y=0") 

    print("\n--- TEST START ---")
    start_pos = get_position()
    if not start_pos: raise ConnectionError("Failed to get start position")
    start_x, start_y = start_pos
    print(f"   => Start Position: X = {start_x:.4f} mm, Y = {start_y:.4f} mm")
    
    for i in range(NUM_MOVES):
        print(f"\n--- Loop {i+1} of {NUM_MOVES} ---")
        
        fire_ttl_pulse()
        
        target_x = start_x + MOVE_DISTANCE_X_MM * (i + 1)
        print(f"  Moving to Target: X={target_x:.4f} and waiting {DWELL_TIME_MS} ms...")
        send_command(f"M X={int(target_x * UNITS_MM_TO_DEVICE)} Y={int(start_y * UNITS_MM_TO_DEVICE)}")
       
        wait_for_idle() 
        # Ждем, пока контроллер сам отработает и движение, и ожидание

        fire_ttl_pulse()
        
        pos_after_move = get_position()
        if pos_after_move:
            start_x, start_y = pos_after_move
        else:
            print("  WARNING: Could not verify position, test may be inaccurate.")
            # Продолжаем с последней известной позиции
            start_x = target_x 

    print("\n--- TEST COMPLETE ---")

except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")

finally:
    if ser and ser.is_open:
        print("\n--- CLEANUP ---")
        send_command("WT X=0 Y=0") 
        ser.close()
        print("\nSerial port closed.")