import serial

def send_command(command):
    with serial.Serial('/dev/ttyUSB0', 9600, timeout=1) as ser:
        ser.write(command.encode())
        response = ser.readline().decode()
        print("收到回應:", response)