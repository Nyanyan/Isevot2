import serial

def start_serial_communication(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Serial communication started on {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error starting serial communication: {e}")
        return None

# Example usage
if __name__ == "__main__":
    port = "COM8"  # Replace with your serial port
    baudrate = 9600  # Replace with your desired baud rate
    serial_connection = start_serial_communication(port, baudrate)

    if serial_connection:
        try:
            # Send 'c' to the serial port
            serial_connection.write(b'c')

            # Wait to receive '0'
            while True:
                received = serial_connection.read(1)  # Read one byte
                if received == b'0':
                    print("Received: '0'")
                    break
        except serial.SerialException as e:
            print(f"Error during communication: {e}")
        finally:
            serial_connection.close()
            print("Serial connection closed.")