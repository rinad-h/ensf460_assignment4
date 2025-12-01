# UART Data Logger & Plotter (Python)

This Python program reads serial data sent from a microcontroller over UART, logs it into a CSV file, and generates two plots:

- LED Intensity (%) vs Time  
- ADC Reading vs Time  

The script waits for a `START_DATA` signal from the microcontroller before logging data, and stops automatically when `STOP_DATA` is received.

---

## Requirements

### Python Version
- Python 3.8 or newer

### Required Python Libraries
Install dependencies with:

```bash
pip install pyserial matplotlib
```

The program uses:
- pyserial (UART communication)
- matplotlib (plotting)
- csv, time, datetime (built-in)

---

## Serial Data Format

Your microcontroller must send UART messages formatted as:

```
START_DATA
timestamp_ms,adc_value,intensity_percent
timestamp_ms,adc_value,intensity_percent
...
STOP_DATA
```

Example:

```
1532,487,72
```

---

## Configuration

Before running, update the COM port and group name at the top of the script:

```python
SERIAL_PORT = 'COM3'      # Change to your COM port
BAUD_RATE = 4800
GROUP_NAME = "Group25"    # Used for naming output files
```

---

## How to Run

1. Connect your microcontroller to your computer via USB.
2. Ensure no programs (PuTTY, MPLAB Data Visualizer, Arduino Serial Monitor) are using the COM port.
3. Open a terminal in the directory where the `.py` file is located.
4. Run:

```bash
python your_script_name.py
```

You will see output such as:

```
Opening serial port COM3 at 4800 baud...
Waiting for START_DATA signal...
Received: START_DATA

=== Data collection started ===
Time: 0.123s, ADC: 512, Intensity: 56%
...
=== STOP_DATA received ===
```

---

## Output Files

The script generates the following files automatically:

### 1. CSV File
```
Group25_data.csv
```

Contains:

| Index | Timestamp (s) | ADC Reading | Intensity (%) |
|-------|----------------|-------------|----------------|
| 1     | 0.123          | 512         | 56             |

### 2. Plot Image
```
Group25_plots.png
```

Exports a figure containing:
- LED Intensity vs Time  
- ADC Reading vs Time  

---

## Plot Display

At the end of execution, a window will open showing both plots.  
The figure is also saved to disk.

---

## Error Handling

The script automatically detects:
- Invalid or unavailable COM port
- Microcontroller not connected
- COM port in use
- Badly formatted UART lines

Example:

```
Serial port error: could not open port 'COM3'
Please check:
1. Is the correct COM port specified?
2. Is the microcontroller connected?
3. Is another program using the COM port?
```

---

## Notes for the Teaching Team

- The script is fully self-contained and requires only Python + two libraries.
- It automatically waits for `START_DATA` and stops at `STOP_DATA`. In order to ensure its starting to save/wrote to csv it must show:
  === Data collection started ===
- Output files are automatically named using the `GROUP_NAME` variable.

