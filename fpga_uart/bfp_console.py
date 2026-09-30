#!/usr/bin/env python3
"""
bfp_console.py - interactive console for manually driving bfp_simd_core
on real FPGA hardware over UART.

Usage:
    pip install --break-system-packages pyserial
    python3 bfp_console.py /dev/ttyUSB0
"""

import sys
import struct
import serial

OP_NAMES = {0: "LOAD", 1: "ADD", 2: "SUB", 3: "MUL", 4: "PASS"}
OP_CODES = {v: k for k, v in OP_NAMES.items()}


def build_command(op, waddr, raddr1, raddr2, lane_enable, imm_exp, imm_mant):
    b = bytearray()
    b.append(op & 0x7)
    b.append(waddr & 0x7)
    b.append(raddr1 & 0x7)
    b.append(raddr2 & 0x7)
    b.append(lane_enable & 0xF)
    b.append(imm_exp & 0xFF)
    for m in imm_mant:
        b += struct.pack('<h', m)
    assert len(b) == 14
    return bytes(b)


def decode_response(resp):
    assert len(resp) == 9
    mants = struct.unpack('<4h', resp[0:8])
    exp = struct.unpack('<b', resp[8:9])[0]
    reals = [m * (2.0 ** exp) for m in mants]
    return mants, exp, reals


def send_and_receive(ser, cmd, timeout_retries=2):
    ser.write(cmd)
    ser.flush()
    resp = ser.read(9)
    if len(resp) != 9:
        # occasional first-command-after-boot glitch; one clean retry
        if timeout_retries > 0:
            return send_and_receive(ser, cmd, timeout_retries - 1)
        raise RuntimeError(f"expected 9 response bytes, got {len(resp)}: {resp!r}")
    return decode_response(resp)


def ask_int(prompt, default, lo, hi):
    raw = input(f"{prompt} [{default}]: ").strip()
    if raw == "":
        return default
    v = int(raw)
    if not (lo <= v <= hi):
        print(f"  out of range ({lo}-{hi}), using {default}")
        return default
    return v


def ask_lanes(prompt, default):
    raw = input(f"{prompt} (e.g. '1111' or blank for {default:04b}): ").strip()
    if raw == "":
        return default
    try:
        return int(raw, 2)
    except ValueError:
        print(f"  couldn't parse '{raw}' as binary, using default {default:04b}")
        return default

def print_result(label, mants, exp, reals):
    print(f"\n  {label}")
    print(f"  mantissas = {list(mants)}   exponent = {exp}")
    for i, r in enumerate(reals):
        print(f"    lane{i}: {r:.4f}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    ser = serial.Serial(
        port, 9600, timeout=5,
        rtscts=False, dsrdtr=False, xonxoff=False,
    )
    ser.setRTS(True)
    ser.setDTR(True)
    print(f"Connected to bfp_simd_core on {port} @ 9600 baud")
    print("(if the very first command looks wrong, just re-run it once --")
    print(" that's a known settling glitch right after a fresh reset)\n")

    while True:
        print("\n" + "=" * 50)
        print("Registers 0-7 available. Operations: LOAD ADD SUB MUL PASS")
        op_name = input("Operation [LOAD]: ").strip().upper() or "LOAD"
        if op_name in ("Q", "QUIT", "EXIT"):
            break
        if op_name not in OP_CODES:
            print(f"  unknown op '{op_name}', options are {list(OP_CODES)}")
            continue
        op = OP_CODES[op_name]

        waddr = ask_int("Destination register (waddr)", 0, 0, 7)

        if op_name == "LOAD":
            raddr1 = raddr2 = 0
            exp = ask_int("Exponent", 0, -128, 127)
            print("Enter 4 mantissa values (one per lane, -32768..32767):")
            mants = []
            for lane in range(4):
                mants.append(ask_int(f"  lane{lane} mantissa", 0, -32768, 32767))
        else:
            raddr1 = ask_int("Source register A (raddr1)", 0, 0, 7)
            raddr2 = ask_int("Source register B (raddr2)", 1, 0, 7)
            exp = 0
            mants = [0, 0, 0, 0]

        lane_enable = ask_lanes("Lane enable mask (4 bits, 1=active)", 0b1111)

        cmd = build_command(op, waddr, raddr1, raddr2, lane_enable, exp, mants)
        try:
            r_mants, r_exp, r_reals = send_and_receive(ser, cmd)
            print_result(
                f"{op_name} -> reg{waddr}"
                + (f" (from reg{raddr1}, reg{raddr2})" if op_name != "LOAD" else ""),
                r_mants, r_exp, r_reals,
            )
        except RuntimeError as e:
            print(f"  ERROR: {e}")

    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
