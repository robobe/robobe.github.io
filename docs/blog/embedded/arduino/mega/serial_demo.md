---
tags:
    - arduino
    - serial
    - cpp
    - python
---

# Communicate with arduino over serial
Using arduino mega open serial port Read/Write data using Cpp and python

## Demo 
Send data from pc to arduino using python and cpp

### Arduino
```cpp title="get data from serial"
#include <Arduino.h>

String incomingMessage = "";  // Store incoming characters
char incomingByte;            // Variable to store each incoming byte
const char terminator = '\n'; 

void setup()
{
    Serial1.begin(9600);
    Serial.begin(9600);

    Serial.println("<Arduino is ready>");
}

//===============

void loop() {
  // Check if there is data available on the serial port
  while (Serial1.available() > 0) {
    // Read the incoming byte
    incomingByte = (char)Serial1.read();
    Serial.print(incomingByte, HEX);
    // Append the incoming byte to the message if it's not the termination character
    if (incomingByte != terminator) {
      incomingMessage += incomingByte;
    } else {
      // If the terminator character is received, the message is complete
      Serial.print("Received message: ");
      Serial.println(incomingMessage);
      
      // Clear the incoming message for the next read
      incomingMessage = "";
    }
  }
}


```

### Python
Send serial data using pyserial package

```python title="send data to arduino"
import time
import serial

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    timeout=1
)


while True:
    data = ser.write("hello\n".encode())
    
    print(data)
    time.sleep(1)

```

### Cpp

```cpp
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <string>

int main() {
    const char* portName = "/dev/ttyUSB0";  // Adjust port name (e.g., "/dev/ttyUSB0" or "/dev/ttyACM0")

    // Open the serial port
    int serialPort = open(portName, O_RDWR);
    if (serialPort == -1) {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }

    // Configure serial port settings
    termios tty;
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting terminal attributes." << std::endl;
        close(serialPort);
        return 1;
    }

    cfsetospeed(&tty, B9600);  // Set output baud rate to 9600
    cfsetispeed(&tty, B9600);  // Set input baud rate to 9600

    tty.c_cflag |= (CLOCAL | CREAD);  // Ignore modem controls, enable reading
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;               // 8 data bits
    tty.c_cflag &= ~PARENB;           // No parity
    tty.c_cflag &= ~CSTOPB;           // 1 stop bit

    // Apply settings
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes." << std::endl;
        close(serialPort);
        return 1;
    }

    // Data to send
    std::string data = "Hello, Arduino!\n";
    ssize_t bytesWritten = write(serialPort, data.c_str(), data.size());

    if (bytesWritten == -1) {
        std::cerr << "Error writing to serial port." << std::endl;
    } else {
        std::cout << "Data sent: " << data << std::endl;
    }

    // Close the serial port
    close(serialPort);
    return 0;
}
```

!!! note "termios"
    The termios library is a set of functions that provides low-level control over the behavior of terminals (serial ports). It is commonly used for configuring settings like baud rate, data bits, parity, and stop bits, which are essential for serial communication.
     

---

## Reference
- [C++ Serial Communication with Arduino](https://youtu.be/uHw7QyL4CM8)
- [SerialTransfer](https://github.com/PowerBroker2/SerialTransfer)