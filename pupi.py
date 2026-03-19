import serial
import time
from typing import Optional

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
MOVE_DISTANCE_X_MM = 0.003
DWELL_TIME_S = 0.5      
NUM_MOVES = 1000

ser: Optional[serial.Serial] = None

def send_command(cmd: str, quiet: bool = False) -> str:
    global ser
    # if not quiet: print(f"  CMD > {cmd}")
    # ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(f"{cmd}\r".encode('ascii')) #\
    response = ser.read_until(b'\r\n').strip() #.decode('ascii')
    # if not quiet: print(f"  RSP < {response}")
    return response

def get_position() -> Optional[tuple[float, float]]:
    response = send_command("W X Y")
    if response and response.startswith(":A"):
        parts = response.split()
        return float(parts[1])/UNITS_MM_TO_DEVICE, float(parts[2])/UNITS_MM_TO_DEVICE
    return None

# def wait_for_idle():
#     """Ждет, пока столик не завершит движение."""
#     print("  Waiting for move to complete...")
#     start_time = time.time()
#     while time.time() - start_time < 10:
#         if send_command("/", quiet=True) == 'N':
#             print("  ...Move complete.")
#             return
#         time.sleep(0.05)
#     raise TimeoutError("Stage did not become idle in 10 seconds!")

# def fire_ttl_pulse():
#     """Вручную генерирует короткий TTL импульс."""
#     print("  >>> Firing manual TTL pulse (START of measurement)! <<<")
#     send_command("TTL Y=1", quiet=True) # Поднять линию
#     send_command("TTL Y=0", quiet=True) # Опустить линию

try:
    print(f"Connecting to {COM_PORT}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(0.2); ser.write(bytes([255, 72])); time.sleep(0.1); ser.read_all()
    print("Connection successful.")

    # print("\n--- TTL SETUP ---")
    # print("Setting TTL to manual mode (TTL Y=0)")
    send_command("TTL Y=0")
    send_command("B X={0} Y={0}")
    print("\n--- TEST START ---")
    response = send_command("W X")
    parts = response.split()
    start_pos = float(parts[1])/UNITS_MM_TO_DEVICE
    # # if not start_pos: raise ConnectionError("Failed to get start position")
    # print( start_pos)
    start_x = -6.6454
    # print(f"   => Start Position: X = {start_x:.4f} mm")
    # send_command(f"M X=1000")
    # send_command("B X Y")
    send_command("RT Y=100 Z=100")
    for i in range(NUM_MOVES):
        
        # print(f"\n--- Loop {i+1} of {NUM_MOVES} ---")
        # send_command("TTL X=1")
        # fire_ttl_pulse()
        
        send_command("TTL Y=1", quiet=True)
        send_command("TTL Y=0", quiet=True)
        
        # print(f"  Dwelling for {DWELL_TIME_S} seconds...")
        # time.sleep(DWELL_TIME_S)
        # print("  ...Dwell complete.")
        
        target_x = start_x + MOVE_DISTANCE_X_MM * (i + 1)
        
        # target_p = start_x + MOVE_DISTANCE_X_MM * (i + 1)
        print(f"  Moving to next point: X={target_x:.4f}")
        send_command(f"M X={int(target_x * UNITS_MM_TO_DEVICE)}")
        # time.sleep(1)
        
        # wait_for_idle()

        # pos_after_move = get_position()
        # if pos_after_move:
        #     start_x, start_y = pos_after_move
        
    print("\n--- TEST COMPLETE ---")

except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")

finally:
    if ser and ser.is_open:
        ser.close()
        print("\nSerial port closed.")