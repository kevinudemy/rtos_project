Create a Python script using pymodbus that reads a Modbus RTU slave with:

- Port: COM3
- Baudrate: 9600
- Data format: 8E1 (even parity, 1 stop bit)
- Slave ID: 7

Register map:
- Use the attached Modbus register map PDF to get the human-readable names.
- Do not guess names; if a name is not present in the PDF, use a generic label (e.g., "Input Reg 30001").

Read:
- Discrete inputs 10001–10004
- Input registers 30001–30007
- Holding registers 40001–40005

Behavior:
- Read holding registers once at startup
- Poll discrete inputs and input registers every 2 seconds
- Print human-readable names + values
- Handle Modbus errors cleanly
- Note: pymodbus uses 0-based addressing in function calls (e.g., 10001 -> address 0).
