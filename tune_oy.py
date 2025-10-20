import serial
import time
from typing import Optional

COM_PORT = "COM4"
BAUD_RATE = 9600
UNITS_MM_TO_DEVICE = 10000

ser: Optional[serial.Serial] = None

def send_command(cmd: str) -> str:
    global ser
    print(f"  CMD > {cmd}")
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(f"{cmd}\r".encode('ascii'))
    response = ser.read_until(b'\r\n').decode('ascii').strip()
    print(f"  RSP < {response}")
    return response

try:
    print(f"Connecting to {COM_PORT}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(0.2)
    ser.write(bytes([255, 72])); time.sleep(0.1); ser.read_all()
    print("Connection successful.\n")

    # текущее значение KP для оси Y
    print("--- Step 1: Querying current KP value for Y-Axis ---")
    response = send_command("KP Y?")
    #  ":A Y=200"
    current_kp = int(response.split('=')[1])
    print(f"\n   => Current KP value for Y is: {current_kp}\n")

    new_kp = int(current_kp * 0.8) # Уменьшаем на 20% для начала
    print(f"Suggested new value: {new_kp}")
    
    # 3. Получаем ввод от пользователя
    print("\nEnter a new KP value for Y-axis (or press Enter to use suggested):")
    user_input = input(f"New KP value [{new_kp}]: ")
    
    if user_input.strip() == "":
        final_kp = new_kp
    else:
        final_kp = int(user_input)

    # 4. Применяем новое значение
    print(f"\n--- Step 3: Applying new KP value ---")
    send_command(f"KP Y={final_kp}")
    
    print("\nNew KP value has been set for this session.")
    # print("If the 'shaking' is gone, you can save this setting permanently.")
    # print("To do so, send the command 'SS Z' (Save Settings).")


except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")

finally:
    if ser and ser.is_open:
        ser.close()
        print("\nSerial port closed.")