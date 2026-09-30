#!/usr/bin/env python3
"""
host.py - talk to the bfp_simd_core running on the VSDSquadron FM over UART.

Usage:
    pip install --break-system-packages pyserial
    python3 host.py /dev/ttyUSB1        # adjust port as needed

Protocol (must match top.v exactly):
  Command  (14 bytes): op, waddr, raddr1, raddr2, lane_enable, imm_exp,
                        imm_mant lane0 lo/hi, lane1 lo/hi, lane2 lo/hi, lane3 lo/hi
  Response (9 bytes):  result_mant lane0 lo/hi, lane1 lo/hi, lane2 lo/hi,
                        lane3 lo/hi, result_exp
"""

import sys
import struct
import serial

OP_LOAD, OP_ADD, OP_SUB, OP_MUL, OP_PASS = 0, 1, 2, 3, 4


def build_command(op, waddr, raddr1, raddr2, lane_enable, imm_exp, imm_mant):
    """imm_mant: list of 4 signed 16-bit ints (lane0..lane3)"""
    b = bytearray()
    b.append(op & 0x7)
    b.append(waddr & 0x7)
    b.append(raddr1 & 0x7)
    b.append(raddr2 & 0x7)
    b.append(lane_enable & 0xF)
    b.append(imm_exp & 0xFF)
    for m in imm_mant:
        b += struct.pack('<h', m)  # little-endian signed 16-bit
    assert len(b) == 14
    return bytes(b)


def decode_response(resp):
    assert len(resp) == 9
    mants = struct.unpack('<4h', resp[0:8])
    exp = struct.unpack('<b', resp[8:9])[0]
    reals = [m * (2.0 ** exp) for m in mants]
    return mants, exp, reals


def send_and_receive(ser, cmd):
    ser.write(cmd)
    resp = ser.read(9)
    if len(resp) != 9:
        raise RuntimeError(f"expected 9 response bytes, got {len(resp)}: {resp!r}")
    return decode_response(resp)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
    ser = serial.Serial(
        port, 9600, timeout=5,
        rtscts=False, dsrdtr=False, xonxoff=False,  # disable all flow control --
        # the FPGA design drives no CTS/DSR line, so if the OS defaults to
        # hardware flow control, writes silently never leave the port
    )
    ser.setRTS(False)
    ser.setDTR(False)

    print(f"Connected to {port} @ 9600 baud")

    # Example: same sequence as the RTL testbenches -----------------
    # LOAD reg0: exp=0, mantissas 30000,-20000,10000,5000
    cmd = build_command(OP_LOAD, waddr=0, raddr1=0, raddr2=0,
                         lane_enable=0b1111, imm_exp=0,
                         imm_mant=[30000, -20000, 10000, 5000])
    mants, exp, reals = send_and_receive(ser, cmd)
    print(f"LOAD reg0 -> mantissas={mants} exp={exp} real={reals}")

    # LOAD reg1: exp=4, mantissa 1250 in every lane (= 20000 real value)
    cmd = build_command(OP_LOAD, waddr=1, raddr1=0, raddr2=0,
                         lane_enable=0b1111, imm_exp=4,
                         imm_mant=[1250, 1250, 1250, 1250])
    mants, exp, reals = send_and_receive(ser, cmd)
    print(f"LOAD reg1 -> mantissas={mants} exp={exp} real={reals}")

    # ADD reg0 + reg1 -> reg2 (requires alignment: exp 0 vs exp 4)
    cmd = build_command(OP_ADD, waddr=2, raddr1=0, raddr2=1,
                         lane_enable=0b1111, imm_exp=0, imm_mant=[0, 0, 0, 0])
    mants, exp, reals = send_and_receive(ser, cmd)
    print(f"ADD  reg0+reg1 -> reg2: mantissas={mants} exp={exp} real={reals}")
    print("  (expect roughly [50000, 0, 30000, 25000])")

    # MUL reg0 * reg1 -> reg3
    cmd = build_command(OP_MUL, waddr=3, raddr1=0, raddr2=1,
                         lane_enable=0b1111, imm_exp=0, imm_mant=[0, 0, 0, 0])
    mants, exp, reals = send_and_receive(ser, cmd)
    print(f"MUL  reg0*reg1 -> reg3: mantissas={mants} exp={exp} real={reals}")
    print("  (expect roughly [6.0e8, -4.0e8, 2.0e8, 1.0e8])")

    ser.close()


if __name__ == "__main__":
    main()
