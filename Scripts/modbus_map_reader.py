from pymodbus.client import ModbusSerialClient
import time

# -----------------------------
# Modbus connection parameters
# -----------------------------
PORT = "COM3"          # Adjust as needed
BAUDRATE = 9600
BYTESIZE = 8
PARITY = "E"           # Even
STOPBITS = 1
TIMEOUT = 1
SLAVE_ID = 7

POLL_INTERVAL_SEC = 2

# -----------------------------
# Register maps (0-based)
# -----------------------------

# Discrete Inputs (FC = 0x02)
DISCRETE_INPUTS = [
    ("VOC Alarm", 0),               # 10001
    ("Min Temp Alarm", 1),          # 10002
    ("Max Temp Alarm", 2),          # 10003
    ("Humidity Alarm", 3),          # 10004
]

# Input Registers (FC = 0x04)
INPUT_REGISTERS = [
    ("VOC Index", 0),               # 30001
    ("Ambient Temperature", 1),     # 30002
    ("Humidity Level", 2),          # 30003
    ("VOC Alarm Count", 3),         # 30004
    ("Min Temp Alarm Count", 4),    # 30005
    ("Max Temp Alarm Count", 5),    # 30006
    ("Humidity Alarm Count", 6),    # 30007
]

# Holding Registers (FC = 0x03)
HOLDING_REGISTERS = [
    ("Sampling Interval (ms)", 0),  # 40001
    ("Max VOC Threshold", 1),       # 40002
    ("Max Temp Threshold", 2),      # 40003
    ("Min Temp Threshold", 3),      # 40004
    ("Max Humidity Threshold", 4),  # 40005
]

# -----------------------------
# Create Modbus client
# -----------------------------
client = ModbusSerialClient(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=BYTESIZE,
    parity=PARITY,
    stopbits=STOPBITS,
    timeout=TIMEOUT,
)

if not client.connect():
    print("ERROR: Failed to connect to Modbus device")
    exit(1)

print("Connected to Modbus RTU slave\n")

# -----------------------------
# Read holding registers once
# -----------------------------
print("Holding Registers (configuration):")
for name, address in HOLDING_REGISTERS:
    response = client.read_holding_registers(
        address=address,
        count=1,
        device_id=SLAVE_ID
    )

    if response.isError():
        print(f"[ERROR] {name}: {response}")
    else:
        print(f"[OK] {name}: {response.registers[0]}")

print("\nStarting Modbus polling...\n")

# -----------------------------
# Poll discrete + input registers
# -----------------------------
try:
    while True:
        print("Discrete Inputs (alarms):")
        for name, address in DISCRETE_INPUTS:
            response = client.read_discrete_inputs(
                address=address,
                count=1,
                device_id=SLAVE_ID
            )

            if response.isError():
                print(f"[ERROR] {name}: {response}")
            else:
                state = response.bits[0]
                print(f"[OK] {name}: {'ACTIVE' if state else 'inactive'}")

        print("\nInput Registers (sensor data):")
        for name, address in INPUT_REGISTERS:
            response = client.read_input_registers(
                address=address,
                count=1,
                device_id=SLAVE_ID
            )

            if response.isError():
                print(f"[ERROR] {name}: {response}")
            else:
                print(f"[OK] {name}: {response.registers[0]}")

        print("-" * 60)
        time.sleep(POLL_INTERVAL_SEC)

except KeyboardInterrupt:
    print("\nStopping Modbus polling...")

finally:
    client.close()
    print("Disconnected")
