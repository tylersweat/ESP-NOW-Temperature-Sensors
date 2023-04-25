import serial

s = serial.Serial('COM10', baudrate=115200, timeout=0.050)
s.flushInput()

with open('data.csv', 'wb') as f:
    while True:
        try:
            f.write(s.readline())
        except KeyboardInterrupt:
            print('Keyboard Interrupt')
            break
        