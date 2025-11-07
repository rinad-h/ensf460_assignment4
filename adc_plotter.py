#File:   adc_plotter.py
#Author: Rinad Hamid, Simar Kandola, Abia Jahangir


import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

#configs
SERIAL_PORT = 'COM3'
BAUD_RATE = 4800
STREAM_DURATION_S = 10  # how long to record for
V_REF = 3.3
ADC_MAX = 1023
SERIAL_TIMEOUT = 0.1
MAX_WAIT_FOR_START = 30  # max seconds to wait for the MCU to say "start"

data_values = []
timestamps = []

def main():
    print("Python ADC Plotter")
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

    ser = None
    try:
        # Try to open the serial port
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=SERIAL_TIMEOUT
        )
        time.sleep(0.5)
        ser.reset_input_buffer()
        print("Connected. Switch to Mode 1 on your device.")
        print("Waiting for 'STREAMING_START' signal...")

        # Wait for the microcontroller to send a start message
        start_wait_time = time.time()
        while True:
            if time.time() - start_wait_time > MAX_WAIT_FOR_START:
                print("ERROR: Timeout waiting for STREAMING_START")
                return
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if line == "STREAMING_START":
                print("Stream started! Collecting data...")
                ser.reset_input_buffer()
                time.sleep(0.1)
                break

        # Start timing the data collection
        start_time = time.perf_counter()
        sample_count = 0
        print(f"Collecting data for {STREAM_DURATION_S} seconds...")

        while True:
            current_time = time.perf_counter()
            elapsed_time = current_time - start_time
            if elapsed_time >= STREAM_DURATION_S:
                print(f"\nTime limit of {STREAM_DURATION_S} seconds reached. Stopping.")
                break

            # Read next serial line
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line or line == "STREAMING_STOP":
                continue

            # Make sure it’s actually a number
            try:
                sample = int(line)
                if sample < 0 or sample > ADC_MAX:
                    print(f"Warning: Out of range ADC value: {sample}")
                    continue
            except ValueError:
                continue

            # Convert ADC reading to voltage and save it
            sample_time = elapsed_time
            voltage = (sample / ADC_MAX) * V_REF
            data_values.append(sample)
            timestamps.append(sample_time)
            sample_count += 1
            print(f"[{sample_time:06.3f}s] Sample #{sample_count:4d}: ADC={sample:4d} ({voltage:.3f} V)")

        # Print a quick summary when done
        print(f"\n{'='*60}")
        if data_values:
            last_val = data_values[-1]
            last_time = timestamps[-1]
            last_volt = (last_val / ADC_MAX) * V_REF
            print("Data collection complete!")
            print(f"Total samples: {len(data_values)}")
            print(f"Time span: {timestamps[0]:.3f}s to {last_time:.3f}s")
            print(f"Final sample: ADC={last_val}, Voltage={last_volt:.3f} V")
        else:
            print("ERROR: No samples collected!")
            return
        print(f"{'='*60}\n")

        # Close serial before plotting/saving
        if ser is not None and ser.is_open:
            ser.close()
            ser = None

        # Turn lists into numpy arrays (makes math easier)
        adc_array = np.array(data_values)
        time_array = np.array(timestamps)
        voltage_array = (adc_array / ADC_MAX) * V_REF

        # Print out a few stats about what we captured
        print("--- Statistics ---")
        print(f"Samples collected: {len(data_values)}")
        print(f"ADC Range: {adc_array.min()} - {adc_array.max()}")
        print(f"Voltage Range: {voltage_array.min():.3f} V - {voltage_array.max():.3f} V")
        print(f"Average ADC: {adc_array.mean():.1f}")
        print(f"Average Voltage: {voltage_array.mean():.3f} V")

        # Check timing consistency (sample rate)
        if len(time_array) > 1:
            intervals = np.diff(time_array)
            print("Sample rate stats:")
            print(f"  Avg interval: {intervals.mean():.3f} s ({1/intervals.mean():.1f} Hz)")
            print(f"  Min interval: {intervals.min():.3f} s")
            print(f"  Max interval: {intervals.max():.3f} s")

        # Save data to CSV with timestamp in the filename
        out_dir = os.path.dirname(__file__) or "."
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(out_dir, f"adc_capture_{timestamp_str}.csv")
        header = "time_s,adc_value,voltage_v"
        np.savetxt(csv_path, np.column_stack((time_array, adc_array, voltage_array)),
                   delimiter=",", header=header, comments='', fmt="%.6f,%d,%.6f")
        print(f"\nSaved {len(data_values)} samples to {csv_path}")

        # Plot ADC values over time
        plt.figure(1, figsize=(12, 5))
        plt.plot(time_array, adc_array, linestyle='-', marker='o', markersize=4, linewidth=1.5)
        plt.title(f"ADC Value vs Time ({len(data_values)} samples)")
        plt.xlabel("Time (s)")
        plt.ylabel("ADC Value (0–1023)")
        plt.grid(True, alpha=0.3)
        plt.ylim(-50, 1074)
        if timestamps:
            plt.xlim(0, max(STREAM_DURATION_S, timestamps[-1]))

        # Plot voltages over time for easier interpretation
        plt.figure(2, figsize=(12, 5))
        plt.plot(time_array, voltage_array, linestyle='-', marker='o', markersize=4, linewidth=1.5, color='orange')
        plt.title(f"Voltage vs Time ({len(data_values)} samples)")
        plt.xlabel("Time (s)")
        plt.ylabel("Voltage (V)")
        plt.grid(True, alpha=0.3)
        plt.ylim(-0.1, V_REF * 1.1)
        if timestamps:
            plt.xlim(0, max(STREAM_DURATION_S, timestamps[-1]))

        print("\nDisplaying plots...")
        plt.show()

    except serial.SerialException as e:
        print(f"\nERROR: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always make sure the port gets closed properly
        if ser is not None and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
