import serial
import csv
import time

# Serial port settings (Modify as needed)
# Notes: left port, long adapter
# Port: /dev/tty.usbserial-58971201781 Serial Port
# python serialReadL.py

SERIAL_PORT = "/dev/tty.usbserial-58971201781"  
BAUD_RATE = 115200

CSV_FILE = "parking_log_driving_1_L.csv"

# Open serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Open (or create) CSV file for logging
with open(CSV_FILE, mode="a", newline="") as file:
    writer = csv.writer(file)
    
    # Write header if the file is empty
    if file.tell() == 0:
        writer.writerow(["Timestamp", "Lat", "Lon", "Angle", "Distance"])

    print("Logging distance data to", CSV_FILE)
    
    try:
        while True:
            # Read a line from Serial
            line = ser.readline().decode("utf-8").strip()
            
            if line:
                try:
                    # Parse the new format: '423652073 -711366075 299 17.01'
                    parts = line.split()
                    if len(parts) == 4:
                        lat, lon, angle, distance = parts[0], parts[1], float(parts[2]), float(parts[3])
                        
                        # Get current timestamp
                        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                        
                        # Append to CSV
                        writer.writerow([timestamp, lat, lon, angle, distance])
                        print(f"{timestamp}, Lat: {lat}, Lon: {lon}, Angle: {angle}, Distance: {distance}")
                    else:
                        print("Invalid data format:", line)
                except ValueError:
                    print("Invalid data:", line)  # Handle bad data gracefully
                    
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
        ser.close()