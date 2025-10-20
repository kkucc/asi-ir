import serial
import time
from typing import Optional

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000
MOVE_DISTANCE_X_MM = 5.0
PULSE_DURATION_MS = 10 # Длительность TTL импульса в миллисекундах

ser: Optional[serial.Serial] = None

def send_command(cmd: str) -> str:
    global ser
    print(f"  CMD > {cmd}")
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(f"{cmd}\r".encode('ascii'))
    response = ser.read_until(b'\r\n').decode('ascii').strip()
    return response

def get_position() -> Optional[tuple[float, float]]:
    response = send_command("W X Y")
    if response and response.startswith(":A"):
        parts = response.split()
        return float(parts[1])/UNITS_MM_TO_DEVICE, float(parts[2])/UNITS_MM_TO_DEVICE
    return None

def wait_for_idle():
    print("  Waiting for move to complete...")
    while True:
        if ser.write(b'/\r') and ser.read(1) == b'N':
            print("  ...Move complete.")
            return
        time.sleep(0.05)

try:
    print(f"Connecting to {COM_PORT}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(0.2)
    ser.write(bytes([255, 72])); time.sleep(0.1); ser.read_all()
    print("Connection successful.")

    print("\n--- TTL SETUP ---")
    print(f"Setting TTL pulse duration to {PULSE_DURATION_MS} ms...")
    send_command(f"RT Y={PULSE_DURATION_MS}")
    print("Enabling 'Pulse on Move Completion' mode (TTL Y=2)...")
    send_command("TTL Y=2")

    print("\n--- TEST START ---")
    print("1. Getting initial position...")
    start_x, start_y = get_position()
    print(f"   => Start Position: X = {start_x:.4f} mm, Y = {start_y:.4f} mm")
    for i in range(5):
        target_x = start_x + MOVE_DISTANCE_X_MM*(i + 1)
        print(f"\n2. Moving X by {MOVE_DISTANCE_X_MM} mm...")
        send_command(f"M X={int(target_x * UNITS_MM_TO_DEVICE)} Y={int(start_y * UNITS_MM_TO_DEVICE)}")
        
        wait_for_idle()

        print("\n>>> TTL PULSE HAS BEEN SENT BY THE CONTROLLER! <<<\n")
        
    print("3. Getting final position...")
    end_x, end_y = get_position()
    print(f"   => Final Position: X = {end_x:.4f} mm, Y = {end_y:.4f} mm")

except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")

finally:
    if ser and ser.is_open:
        print("\n--- CLEANUP ---")
        print("Disabling TTL mode (TTL Y=0)...")
        send_command("TTL Y=0") # Возвращаем выход в состояние LOW
        ser.close()
        print("Serial port closed.")