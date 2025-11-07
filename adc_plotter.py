import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = 'COM3'  # !! IMPORTANT: Change this to your COM port !!
BAUD_RATE = 4800    # This MUST match your C code's UART speed. Increase if your device supports it.
STREAM_DURATION_S = 10  # Collect data for 10 seconds
SAMPLE_INTERVAL_S = 1.0  # one sample per second
V_REF = 3.3  # Reference voltage (e.g., 3.3V). Adjust if yours is different.
ADC_MAX = 1023  # 10-bit ADC

# Tuning to increase samples collected within WINDOW:
SERIAL_TIMEOUT = 0.01  # seconds; small timeout to avoid long blocking reads
PRINT_EVERY = 1        # print every sample (one per second)
MAX_SAMPLES = None     # still unused when using SAMPLE_INTERVAL_S

# --- Globals ---
data_values = []
timestamps = []

def main():
    print("Python ADC Plotter")
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

    ser = None
    try:
        # Configure serial port
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=SERIAL_TIMEOUT  # small timeout for responsive reads
        )
        print("Connected. Please switch to Mode 1 on your device.")
        print("Waiting for 'STREAMING_START' signal...")

        # --- 1. Synchronization ---
        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if line == "STREAMING_START":
                    print("Stream started! Collecting data...")
                    break
            except Exception as e:
                # Non-fatal; keep waiting
                print(f"Warning while waiting for sync: {e}")

        # --- 2. Data Collection (one sample per second) ---
        start_time = time.perf_counter()
        end_time = start_time + STREAM_DURATION_S
        num_samples = int(STREAM_DURATION_S // SAMPLE_INTERVAL_S)
        last_valid_sample = None

        for i in range(num_samples):
            target_time = start_time + i * SAMPLE_INTERVAL_S

            # Sleep until close to the target time (short sleeps for responsiveness)
            while time.perf_counter() < target_time:
                time.sleep(0.001)

            # Try to read a valid integer sample within a short window
            sample = None
            read_deadline = time.perf_counter() + 0.2  # allow up to 200 ms to get a sample
            while time.perf_counter() < read_deadline:
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    sample = int(line)
                    break
                except ValueError:
                    # not an integer, keep reading until deadline
                    continue
                except Exception as e:
                    print(f"Error reading from serial: {e}")
                    break

            # Fallback to last valid sample if none read in window
            if sample is None:
                if last_valid_sample is not None:
                    sample = last_valid_sample
                else:
                    sample = 0  # no previous sample, use 0

            current_time = time.perf_counter() - start_time
            data_values.append(sample)
            timestamps.append(current_time)
            last_valid_sample = sample

            if len(data_values) % PRINT_EVERY == 0:
                print(f"[{current_time:05.2f}s] Samples collected: {len(data_values)}")

        print(f"\nFinished collecting {len(data_values)} samples.")

        # --- Ensure serial closed ---
        ser.close()
        ser = None

        # --- 3. Data Processing and Plotting ---
        if not data_values:
            print("No data collected. Exiting.")
            return

        adc_array = np.array(data_values)
        time_array = np.array(timestamps)
        voltage_array = (adc_array / ADC_MAX) * V_REF

        # Save CSV
        out_dir = os.path.dirname(__file__) or "."
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(out_dir, f"adc_capture_{timestamp_str}.csv")
        header = "time_s,adc_value,voltage_v"
        np.savetxt(csv_path, np.column_stack((time_array, adc_array, voltage_array)),
                   delimiter=",", header=header, comments='', fmt="%.6f, %d, %.6f")
        print(f"Saved {len(data_values)} samples to {csv_path}")

        # --- Plot 1: ADC Value vs. Time ---
        plt.figure(1)
        plt.plot(time_array, adc_array, linestyle='-', marker='o')
        plt.title(f"ADC Value vs. Time ({STREAM_DURATION_S}s)")
        plt.xlabel("Time (s)")
        plt.ylabel("ADC Value (0-1023)")
        plt.grid(True)
        plt.ylim(0, 1024)

        # --- Plot 2: Voltage vs. Time ---
        plt.figure(2)
        plt.plot(time_array, voltage_array, linestyle='-', marker='o', color='orange')
        plt.title(f"Voltage vs. Time ({STREAM_DURATION_S}s)")
        plt.xlabel("Time (s)")
        plt.ylabel(f"Voltage (V)")
        plt.grid(True)
        plt.ylim(0, V_REF * 1.05)

        print("Displaying plots. Close both plot windows to exit.")
        plt.show()

    except serial.SerialException as e:
        print(f"\n--- ERROR ---")
        print(f"Could not open serial port {SERIAL_PORT}.")
        print("Please check the following:")
        print(f"1. Is the device plugged in?")
        print(f"2. Is '{SERIAL_PORT}' the correct COM port? (Check Device Manager)")
        print(f"3. Is the terminal program (RealTerm) closed?")
        print(f"Error details: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser is not None and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
