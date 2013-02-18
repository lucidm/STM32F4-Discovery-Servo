import serial, time, math

seq = [(90,-90), (-90,90), (60,-60), (-60,60), (30,-30), (-30,30), (15,-15), (-15,15), (0,0),
       (-15,15), (15,-15),(-30,30),(30,-30),(-60,60),(60,-60),(-90,90),(90,-90),(0,0)]

ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,timeout=0)
ts = 0.15

ser.write("freq 47\r\n")

for x in range(100):
    for i in seq:
        ser.write("smov {s1} {s2}\r\n".format(s1=i[0], s2=i[1]))
        r = ser.read(ser.inWaiting())
        ser.flush()
        time.sleep(ts)
        print r
