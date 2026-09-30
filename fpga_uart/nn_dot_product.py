#!/usr/bin/env python3
"""
nn_dot_product.py - a tiny neural-network-style dot product (weights . inputs,
the core operation inside every dense/linear layer), computed three ways:

  1. TRUE     - exact float64 math (ground truth)
  2. FIXED    - naive Q16.16 fixed-point, simulated on the host (mimics the
                design1 fixed_point_simd core's actual arithmetic, including
                its real wraparound-on-overflow failure mode)
  3. HARDWARE - your actual bfp_simd_core, running live on the FPGA. Weights
                and inputs are quantized into 4-element blocks (one shared
                exponent per block, exactly matching the hardware's real
                block-floating-point register format), multiplied on real
                silicon, and the products are summed on the host to form
                the final dot product -- the same split between "hardware
                does the MAC, host does the reduction" used in real
                accelerators.

This models the real reason block-floating-point / microscaling formats are
used in ML accelerators: neural network weights and activations span a wide
dynamic range within a single layer, and naive fixed-point silently
overflows on the large values while wasting precision on the small ones.

Usage:
    python3 nn_dot_product.py /dev/ttyUSB0 [--n 16] [--seed 1]
"""

import sys
import struct
import argparse
import random
import serial

OP_LOAD, OP_MUL = 0, 3


# ---------------------------------------------------------------- UART link

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
    mants = struct.unpack('<4h', resp[0:8])
    exp = struct.unpack('<b', resp[8:9])[0]
    reals = [m * (2.0 ** exp) for m in mants]
    return mants, exp, reals


def send_and_receive(ser, cmd, retries=2):
    ser.write(cmd)
    ser.flush()
    resp = ser.read(9)
    if len(resp) != 9:
        if retries > 0:
            return send_and_receive(ser, cmd, retries - 1)
        raise RuntimeError(f"expected 9 bytes, got {len(resp)}: {resp!r}")
    return decode_response(resp)


# ------------------------------------------------------------- quantization

def quantize_block_bfp(values):
    """Quantize up to 4 real values into a shared-exponent BFP block,
    exactly matching the hardware's register format: one 8-bit exponent
    covering all 4 int16 mantissas in that register."""
    vals = list(values) + [0.0] * (4 - len(values))
    max_abs = max(abs(v) for v in vals) or 1.0
    # choose exponent so the largest magnitude uses close to the full
    # int16 range (headroom to avoid clipping on rounding)
    import math
    exp = math.ceil(math.log2(max_abs / 32000.0)) if max_abs > 0 else 0
    exp = max(-128, min(127, exp))
    scale = 2.0 ** (-exp)
    mants = [max(-32768, min(32767, round(v * scale))) for v in vals]
    return mants, exp


def q16_16_multiply_accumulate(weights, inputs):
    """Simulate the SAME real arithmetic as the design1 fixed-point core:
    Q16.16 fixed point, 32-bit accumulator, silent two's-complement
    wraparound on overflow -- the actual documented failure mode from the
    original accuracy benchmark."""
    Q = 65536  # 2^16
    acc = 0
    for w, x in zip(weights, inputs):
        w_fp = int(round(w * Q))
        x_fp = int(round(x * Q))
        prod = (w_fp * x_fp) >> 16  # Q16.16 * Q16.16 -> Q16.16
        acc = (acc + prod) & 0xFFFFFFFF
        if acc >= 0x80000000:  # interpret as signed 32-bit, matching hardware
            acc -= 0x100000000
    return acc / Q


# -------------------------------------------------------------------- main

def run_hardware_dot_product(ser, weights, inputs, verbose=True):
    total = 0.0
    n = len(weights)
    for start in range(0, n, 4):
        w_chunk = weights[start:start + 4]
        x_chunk = inputs[start:start + 4]
        w_mants, w_exp = quantize_block_bfp(w_chunk)
        x_mants, x_exp = quantize_block_bfp(x_chunk)

        cmd = build_command(OP_LOAD, waddr=0, raddr1=0, raddr2=0,
                             lane_enable=0b1111, imm_exp=w_exp, imm_mant=w_mants)
        send_and_receive(ser, cmd)

        cmd = build_command(OP_LOAD, waddr=1, raddr1=0, raddr2=0,
                             lane_enable=0b1111, imm_exp=x_exp, imm_mant=x_mants)
        send_and_receive(ser, cmd)

        cmd = build_command(OP_MUL, waddr=2, raddr1=0, raddr2=1,
                             lane_enable=0b1111, imm_exp=0, imm_mant=[0, 0, 0, 0])
        mants, exp, reals = send_and_receive(ser, cmd)

        chunk_len = len(w_chunk)
        chunk_sum = sum(reals[:chunk_len])
        total += chunk_sum
        if verbose:
            print(f"  chunk {start//4}: w={w_chunk} x={x_chunk}")
            print(f"    hw products = {[round(r,2) for r in reals[:chunk_len]]}"
                  f"  (exp={exp})  partial sum = {chunk_sum:.4f}")
    return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--n", type=int, default=16, help="number of weights/inputs")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    random.seed(args.seed)

    # realistic-ish neural net weight/activation distribution: mostly small
    # values with a few much larger outliers, spanning several orders of
    # magnitude -- exactly the pattern that breaks naive fixed-point
    weights, inputs = [], []
    for _ in range(args.n):
        if random.random() < 0.4:
            w = random.uniform(-2000, 2000)    # much larger, more frequent outliers
        else:
            w = random.uniform(-2.0, 2.0)
        x = random.uniform(-50.0, 50.0)
        weights.append(w)
        inputs.append(x)

    print(f"Neuron with {args.n} inputs")
    print(f"weights = {[round(w,3) for w in weights]}")
    print(f"inputs  = {[round(x,3) for x in inputs]}")

    true_result = sum(w * x for w, x in zip(weights, inputs))
    fixed_result = q16_16_multiply_accumulate(weights, inputs)

    print(f"\n=== Computing dot product on real hardware ({args.port}) ===")
    ser = serial.Serial(args.port, 9600, timeout=5,
                         rtscts=False, dsrdtr=False, xonxoff=False)
    ser.setRTS(True)
    ser.setDTR(True)
    hw_result = run_hardware_dot_product(ser, weights, inputs)
    ser.close()

    print("\n" + "=" * 60)
    print(f"{'Method':<12} {'Result':>18}   Error vs true")
    print("-" * 60)
    print(f"{'TRUE':<12} {true_result:>18.4f}   -")
    fe = abs(fixed_result - true_result) / max(abs(true_result), 1e-9) * 100
    print(f"{'FIXED':<12} {fixed_result:>18.4f}   {fe:.2f}%  (Q16.16, host-simulated)")
    he = abs(hw_result - true_result) / max(abs(true_result), 1e-9) * 100
    print(f"{'HARDWARE':<12} {hw_result:>18.4f}   {he:.2f}%  (bfp_simd_core, real silicon)")
    print("=" * 60)


if __name__ == "__main__":
    main()
