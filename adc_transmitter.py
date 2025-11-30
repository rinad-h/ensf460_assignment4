import serial
import csv
import matplotlib.pyplot as plt
import time
from datetime import datetime

# Configuration
SERIAL_PORT = 'COM3'  
BAUD_RATE = 4800
GROUP_NAME = "Group25"  
CSV_FILENAME = f"{GROUP_NAME}_data.csv"

def main():
    # Lists to store data
    timestamps = []
    adc_readings = []
    intensity_levels = []
    
    print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        # Open serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        
        print("Waiting for START_DATA signal...")
        
        # Wait for START_DATA signal
        data_collection_started = False
        while not data_collection_started:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"Received: {line}")
                
                if "START_DATA" in line:
                    data_collection_started = True
                    print("\n=== Data collection started ===\n")
        
        # Collect data until STOP_DATA received (microcontroller handles 60s timing)
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Check for STOP_DATA signal
                if "STOP_DATA" in line:
                    print("\n=== STOP_DATA received ===")
                    break
                
                # Skip empty lines or status lines
                if not line or "MODE" in line:
                    continue
                
                # Parse data line: TIMESTAMP,ADC_VALUE,INTENSITY_PERCENT
                try:
                    # Remove any null bytes or extra whitespace
                    line = line.replace('\x00', '').strip()
                    
                    parts = line.split(',')
                    if len(parts) == 3:
                        timestamp_ms = int(parts[0].strip())
                        adc_value = int(parts[1].strip())
                        intensity = int(parts[2].strip())
                        
                        # Convert timestamp from milliseconds to seconds
                        timestamp_sec = timestamp_ms / 1000.0
                        
                        # Store data
                        timestamps.append(timestamp_sec)
                        adc_readings.append(adc_value)
                        intensity_levels.append(intensity)
                        
                        print(f"Time: {timestamp_sec:.3f}s, ADC: {adc_value}, Intensity: {intensity}%")
                    
                except (ValueError, IndexError) as e:
                    print(f"Error parsing line: {line} - {e}")
                    continue
        
        # Close serial connection
        ser.close()
        print("\nSerial connection closed.")
        
        # Check if we have data
        if len(timestamps) == 0:
            print("No data collected. Exiting.")
            return
        
        print(f"\nTotal data points collected: {len(timestamps)}")
        print(f"Time range: {timestamps[0]:.3f}s to {timestamps[-1]:.3f}s")
        
        # Save data to CSV
        print(f"\nSaving data to {CSV_FILENAME}...")
        with open(CSV_FILENAME, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(['Index', 'Timestamp (s)', 'ADC Reading', 'Intensity (%)'])
            # Write data
            for i in range(len(timestamps)):
                writer.writerow([i+1, f"{timestamps[i]:.3f}", adc_readings[i], intensity_levels[i]])
        
        print(f"Data saved to {CSV_FILENAME}")
        
        # Generate plots
        print("\nGenerating plots...")
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot 1: Intensity vs Time
        ax1.plot(timestamps, intensity_levels, 'b-', linewidth=1.5)
        ax1.set_xlabel('Time (seconds)', fontsize=12)
        ax1.set_ylabel('Intensity (%)', fontsize=12)
        ax1.set_title('LED Intensity Level vs Time', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-5, 105)
        ax1.set_xlim(0, 60)  # Set x-axis to full 60 seconds
        
        # Plot 2: ADC Reading vs Time
        ax2.plot(timestamps, adc_readings, 'r-', linewidth=1.5)
        ax2.set_xlabel('Time (seconds)', fontsize=12)
        ax2.set_ylabel('ADC Reading', fontsize=12)
        ax2.set_title('ADC Reading vs Time', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(-50, 1100)
        ax2.set_xlim(0, 60)  # Set x-axis to full 60 seconds
        
        plt.tight_layout()
        
        # Save the plot
        plot_filename = f"{GROUP_NAME}_plots.png"
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plots saved to {plot_filename}")
        
        # Display the plot
        plt.show()
        
        print("\n=== Data collection and plotting complete ===")
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Please check:")
        print("1. Is the correct COM port specified?")
        print("2. Is the microcontroller connected?")
        print("3. Is another program using the COM port?")
    except KeyboardInterrupt:
        print("\n\nData collection interrupted by user.")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
