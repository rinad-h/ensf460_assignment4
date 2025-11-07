import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = 'COM3'   # Change to your COM port
BAUD_RATE = 4800       # Must match your C code UART baud rate
STREAM_DURATION_S = 10 # Assignment requires 10-second window
V_REF = 3.3
ADC_MAX = 1023
SERIAL_TIMEOUT = 0.1   # Serial read timeout in seconds
GRACE_PERIOD = 0.5     # Extra time to catch last sample after 10s

# --- Globals ---
data_values = []
timestamps = []

def main():
    print("Python ADC Plotter")
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

    ser = None
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=SERIAL_TIMEOUT
        )
        print("Connected. Switch to Mode 1 on your device.")
        print("Waiting for 'STREAMING_START' signal...")

        # --- Wait for STREAMING_START from microcontroller ---
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if line == "STREAMING_START":
                print("Stream started. Collecting data...")
                break

        # --- Collect data for 10s ---
        start_time = time.perf_counter()
        window_end_time = start_time + STREAM_DURATION_S

        while True:
            current_time = time.perf_counter()
            if current_time > window_end_time + GRACE_PERIOD:
                print("\nTime window + grace period expired.")
                break

            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line or line in ["STREAMING_START", "STREAMING_STOP"]:
                continue

            try:
                sample = int(line)
            except ValueError:
                continue

            elapsed_time = current_time - start_time
            voltage = (sample / ADC_MAX) * V_REF

            data_values.append(sample)
            timestamps.append(elapsed_time)

            print(f"[{elapsed_time:05.2f}s] ADC={sample:4d} ({voltage:.3f} V)")

        # --- Final sample info ---
        if data_values:
            last_val = data_values[-1]
            last_time = timestamps[-1]
            last_volt = (last_val / ADC_MAX) * V_REF
            print(f"\nFinal recorded sample: t={last_time:.2f}s, ADC={last_val}, Voltage={last_volt:.3f} V")
        else:
            print("\nNo samples collected.")

        # --- Close serial port ---
        if ser.is_open:
            ser.close()
            ser = None

        # --- Convert to numpy arrays ---
        adc_array = np.array(data_values)
        time_array = np.array(timestamps)
        voltage_array = (adc_array / ADC_MAX) * V_REF

        # --- Statistics ---
        if len(data_values) > 0:
            print("\n--- Statistics ---")
            print(f"Samples collected: {len(data_values)}")
            print(f"ADC Range: {adc_array.min()} - {adc_array.max()}")
            print(f"Voltage Range: {voltage_array.min():.3f} V - {voltage_array.max():.3f} V")
            print(f"Average ADC: {adc_array.mean():.1f}")
            print(f"Average Voltage: {voltage_array.mean():.3f} V")
            if len(time_array) > 1:
                intervals = np.diff(time_array)
                print(f"Average sample interval: {intervals.mean():.3f} s")

        # --- Save CSV ---
        out_dir = os.path.dirname(__file__) or "."
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(out_dir, f"adc_capture_{timestamp_str}.csv")
        header = "time_s,adc_value,voltage_v"
        np.savetxt(csv_path, np.column_stack((time_array, adc_array, voltage_array)),
                   delimiter=",", header=header, comments='', fmt="%.6f,%d,%.6f")
        print(f"\nSaved {len(data_values)} samples to {csv_path}")

        # --- Plot ADC vs Time ---
        plt.figure(1, figsize=(10, 5))
        plt.plot(time_array, adc_array, linestyle='-', marker='o', markersize=8, linewidth=2)
        plt.title(f"ADC Value vs Time ({len(data_values)} samples)")
        plt.xlabel("Time (s)")
        plt.ylabel("ADC Value (0-1023)")
        plt.grid(True, alpha=0.3)
        plt.ylim(-50, 1074)
        plt.xlim(0, STREAM_DURATION_S)

        # --- Plot Voltage vs Time ---
        plt.figure(2, figsize=(10, 5))
        plt.plot(time_array, voltage_array, linestyle='-', marker='o', markersize=8, linewidth=2, color='orange')
        plt.title(f"Voltage vs Time ({len(data_values)} samples)")
        plt.xlabel("Time (s)")
        plt.ylabel("Voltage (V)")
        plt.grid(True, alpha=0.3)
        plt.ylim(-0.1, V_REF * 1.1)
        plt.xlim(0, STREAM_DURATION_S)

        print("Displaying plots...")
        plt.show()

    except serial.SerialException as e:
        print(f"\nERROR: Could not open serial port {SERIAL_PORT}.")
        print("Check device connection, COM port, and ensure no other program is using it.")
        print(f"Details: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
