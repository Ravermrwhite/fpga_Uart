#!/usr/bin/env python3
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

ser = serial.Serial(
    port, 9600, timeout=2,
    rtscts=False, dsrdtr=False, xonxoff=False,
)
ser.setRTS(True)
ser.setDTR(True)
time.sleep(0.5)

print(f"Opened {port}")
print(f"  rtscts={ser.rtscts} dsrdtr={ser.dsrdtr} xonxoff={ser.xonxoff}")
print(f"  RTS={ser.rts} DTR={ser.dtr}")

cmd = bytes([0, 0, 0, 0, 0b1111, 0,
             48, 117,
             224, 177,
             16, 39,
             136, 19])

print(f"\nSending {len(cmd)} bytes: {list(cmd)}")
n = ser.write(cmd)
ser.flush()
print(f"ser.write() reported {n} bytes written")

time.sleep(1.0)
waiting = ser.in_waiting
print(f"Bytes waiting in input buffer: {waiting}")

if waiting > 0:
    data = ser.read(waiting)
    print(f"Read: {list(data)}")
else:
    print("Nothing received at all within 1s after send.")
    print("Trying a longer wait...")
    time.sleep(3.0)
    waiting = ser.in_waiting
    print(f"Bytes waiting after 3 more seconds: {waiting}")
    if waiting > 0:
        data = ser.read(waiting)
        print(f"Read: {list(data)}")

ser.close()

# Send the SAME command again -- tests whether the first response was
# affected by reset/pipeline settling right after a fresh flash.
print("\nSending the SAME command a second time...")
n = ser.write(cmd)
ser.flush()
time.sleep(1.0)
waiting = ser.in_waiting
print(f"Bytes waiting in input buffer: {waiting}")
if waiting > 0:
    data = ser.read(waiting)
    print(f"Read (2nd attempt): {list(data)}")
else:
    time.sleep(3.0)
    waiting = ser.in_waiting
    if waiting > 0:
        data = ser.read(waiting)
        print(f"Read (2nd attempt): {list(data)}")
