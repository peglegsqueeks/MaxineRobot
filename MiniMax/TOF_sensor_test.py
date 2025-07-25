import serial

PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

def format_sensor_output(distances):
    return "\t".join([f"Sensor {i+1} ({d} mm)" for i, d in enumerate(distances)])

def main():
    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
            print("Reading from TeraRanger Multiflex on /dev/ttyACM0...")
            while True:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith("MF"):
                    line = line[2:].strip()

                try:
                    sensor_distances = list(map(int, line.split('\t')))

                    if len(sensor_distances) == 8:
                        # Print nicely labeled output
                        print(format_sensor_output(sensor_distances))

                        # Now sensor_distances is a pure numeric list, e.g.:
                        # [ -1, -1, 473, 173, 143, 312, -1, -1 ]
                        # You can access Sensor 1 as sensor_distances[0], etc.
                        
                    else:
                        print(f"[Warning] Expected 8 values, got {len(sensor_distances)}: {line}")
                except ValueError:
                    print(f"[Error] Could not parse: {line}")

    except serial.SerialException as e:
        print(f"[Serial Error] {e}")

if __name__ == "__main__":
    main()
