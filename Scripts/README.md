# Modbus Register Map Reader (Python)

This Python script provides a simple **Modbus RTU master** example used to read and display data from the STM32-based Modbus RTU slave implemented in this course.

It is intended as a **development and validation tool**, allowing you to:

- Verify Modbus RTU communication
- Inspect live sensor values
- Confirm register mappings
- Observe alarm states and counters

This script is **not required** to complete the course. It is provided as an optional helper for testing, validation, and experimentation.

---

## Requirements

- **Python 3.x**
- **pymodbus**
- **pyserial**

Install dependencies using pip:

```bash
pip install pymodbus pyserial
```

---

## Modbus Connection Parameters

The following parameters are defined at the top of the script.  
Update them to match your local setup and embedded configuration:

```python
PORT = "COM3"        # Example: COM3 (Windows) or /dev/ttyUSB0 (Linux)
BAUDRATE = 9600
BYTESIZE = 8
PARITY = "E"         # Even parity
STOPBITS = 1
SLAVE_ID = 7
```

Ensure these values match the UART and Modbus RTU configuration on the STM32 device.

---

## Register Map Assumptions

This script assumes the Modbus register layout used throughout the course project.

### Discrete Inputs (Function Code 0x02)

Discrete inputs represent read-only alarm states:

| Address | Description |
|--------:|-------------|
| 10001   | VOC Alarm Active |
| 10002   | Minimum Temperature Alarm Active |
| 10003   | Maximum Temperature Alarm Active |
| 10004   | Humidity Alarm Active |

---

### Input Registers (Function Code 0x04)

Input registers represent live sensor values and alarm counters:

| Address | Description |
|--------:|-------------|
| 30001   | VOC Index |
| 30002   | Ambient Temperature |
| 30003   | Humidity Level |
| 30004   | VOC Alarm Count |
| 30005   | Minimum Temperature Alarm Count |
| 30006   | Maximum Temperature Alarm Count |
| 30007   | Humidity Alarm Count |

---

### Holding Registers (Function Code 0x03)

Holding registers are configuration parameters and are read once at startup:

| Address | Description |
|--------:|-------------|
| 40001   | Sampling Interval (ms) |
| 40002   | Maximum VOC Threshold |
| 40003   | Maximum Temperature Threshold |
| 40004   | Minimum Temperature Threshold |
| 40005   | Maximum Humidity Threshold |

---

## Script Behavior

The script performs the following actions:

1. Connects to the Modbus RTU slave over a serial interface
2. Reads all holding registers once at startup
3. Periodically polls:
   - Discrete inputs
   - Input registers
4. Prints values and Modbus errors to the terminal
5. Continues running until interrupted

---

## Running the Script

From the directory containing the script:

```bash
python modbus_map_reader.py
```

---

## Stopping the Script

Press:

```
Ctrl + C
```

This interrupts the Python process and stops Modbus polling.  
The serial connection is then closed cleanly.

---

## Notes

- This script mirrors the Modbus register structure used throughout the course.
- If the embedded application or register map changes, update this script accordingly.
- The script is intentionally kept simple and readable for learning purposes.
- It is not intended to be a production Modbus master implementation.
