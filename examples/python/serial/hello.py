import serial
baudrate = 9600
serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=baudrate,
                                bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
serialString = ""
# serialPort.write(bytes.fromhex("A551F6"))
serialPort.write(bytes.fromhex("A555FA"))

while True:
    try:
        serialString = serialPort.read()
        print(serialString)
    except KeyboardInterrupt:
        break

serialPort.close()