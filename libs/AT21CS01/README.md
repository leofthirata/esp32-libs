# Requires
- PlatformIO with Arduino Framework
- ESP32 dev kit and AT21CS01 at pin 12
- External PULL-UP resistor is not required. The driver will set the port to output open drain with internal pull-up.

# How to use
- Open the `writeRead` folder under `/test` with PlatformIO;
- Edit `upload_port` and `monitor_port` in `platformio.ini`;
- `Upload and Monitor`.

# Expected
- Log output
    ```
    AT21CS01 ESP32 arduino driver
        Slave Address = 110b
        Standard Speed ACK
    Memory arrange:
    00 01 02 03 04 05 06 07 
    08 09 0A 0B 0C 0D 0E 0F 
    10 11 12 13 14 15 16 17 
    18 19 1A 1B 1C 1D 1E 1F 
    20 21 22 23 24 25 26 27 
    28 29 2A 2B 2C 2D 2E 2F 
    30 31 32 33 34 35 36 37 
    38 39 3A 3B 3C 3D 3E 3F 
    40 41 42 43 44 45 46 47 

    Wait 5s to try reading
    Serial Number:
        Data Read = 0xA0 0xDF 0x70 0x89 0x02 0x00 0x00 0xF0

    72 bytes read from Memory:
        Data Read = 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07
        Data Read = 0x08 0x09 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F
        Data Read = 0x10 0x11 0x12 0x13 0x14 0x15 0x16 0x17
        Data Read = 0x18 0x19 0x1A 0x1B 0x1C 0x1D 0x1E 0x1F
        Data Read = 0x20 0x21 0x22 0x23 0x24 0x25 0x26 0x27
        Data Read = 0x28 0x29 0x2A 0x2B 0x2C 0x2D 0x2E 0x2F
        Data Read = 0x30 0x31 0x32 0x33 0x34 0x35 0x36 0x37
        Data Read = 0x38 0x39 0x3A 0x3B 0x3C 0x3D 0x3E 0x3F
        Data Read = 0x40 0x41 0x42 0x43 0x44 0x45 0x46 0x47
    ....
    ```
    - Print AT21CS01 slave address;
    - Change connection to SLOW;
    - Write 72 bytes to memory;
    - Loop: 
      - Read serial number; 
      - Read 72 bytes from memory;
      - Blink LED;
