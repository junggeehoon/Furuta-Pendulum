import serial
import csv

# Adjust these for your environment:
SERIAL_PORT = '/dev/tty.wchusbserial110'  # e.g. '/dev/ttyUSB0' on Linux
BAUD_RATE = 115200
CSV_FILENAME = 'current.csv'


def main():
    # Open the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # Open the CSV file in write mode
    with open(CSV_FILENAME, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)

        # Optional: write a header row
        writer.writerow(["Time (s)", "Desired Q (A)", "Measured Q (A)"])

        print(f"Reading time,velocity data from {SERIAL_PORT} at {BAUD_RATE} baud...")
        print(f"Saving to {CSV_FILENAME}. Press Ctrl+C to stop.")

        try:
            while True:
                # Read one line from serial
                line = ser.readline().decode('utf-8').strip()

                if line:
                    # Split into two parts
                    parts = line.split(',')

                    if len(parts) == 3:
                        time, desired_q, measured_q = parts
                        writer.writerow([time, desired_q, measured_q])
                        print(time, desired_q, measured_q)

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            ser.close()


if __name__ == '__main__':
    main()