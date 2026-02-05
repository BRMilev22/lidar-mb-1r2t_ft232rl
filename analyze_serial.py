import serial
import time

PORT = '/dev/cu.usbserial-A5069RR4'
BAUDRATE = 153600

def main():
    print(f"Opening {PORT} at {BAUDRATE} baud...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(0.5)
    
    print("Reading data for 5 seconds...")
    print("="*70)
    
    start = time.time()
    data = b''
    while time.time() - start < 5:
        chunk = ser.read(512)
        if chunk:
            data += chunk
    
    ser.close()
    
    print(f"Received {len(data)} bytes\n")
    
    print("Parsing packets (AA 55 header):")
    print("="*70)
    
    i = 0
    packet_num = 0
    while i < len(data) - 10:
        # Look for header
        if data[i] == 0xAA and data[i+1] == 0x55:
            packet_num += 1
            
            lidar_type = data[i+2]
            num_measurements = data[i+3]
            
            # Start angle (little endian)
            start_angle = data[i+4] | (data[i+5] << 8)
            # End angle (little endian)  
            end_angle = data[i+6] | (data[i+7] << 8)
            # Unknown bytes
            unknown1 = data[i+8]
            unknown2 = data[i+9]
            
            print(f"\nPacket #{packet_num} at offset {i}")
            print(f"  Header:        AA 55")
            print(f"  Lidar Type:    0x{lidar_type:02X} ({lidar_type})")
            print(f"  Measurements:  {num_measurements}")
            print(f"  Start Angle:   {start_angle} (raw) = {start_angle/100:.2f}°")
            print(f"  End Angle:     {end_angle} (raw) = {end_angle/100:.2f}°")
            print(f"  Unknown:       0x{unknown1:02X} 0x{unknown2:02X}")
            
            # Print first few measurements
            print(f"  Data bytes (first 15 measurements):")
            data_start = i + 10
            for m in range(min(15, num_measurements)):
                offset = data_start + m * 3
                if offset + 2 < len(data):
                    quality = data[offset]
                    dist_low = data[offset + 1]
                    dist_high = data[offset + 2]
                    distance = dist_low | (dist_high << 8)
                    print(f"    [{m:2d}] Quality: {quality:3d}  Distance: {distance:5d} mm  "
                          f"(raw: {quality:02X} {dist_low:02X} {dist_high:02X})")
            
            # Move to next potential packet
            packet_size = 10 + num_measurements * 3
            i += packet_size
            
            if packet_num >= 10:
                print(f"\n... showing first 10 packets")
                break
        else:
            i += 1
    
    print("\n" + "="*70)
    print("Analysis complete!")

if __name__ == "__main__":
    main()
