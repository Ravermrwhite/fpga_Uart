#!/usr/bin/env python3
"""
webcam_motion.py - real-world plug-and-play demo: your laptop's webcam
feeds real, live, varying brightness data into bfp_simd_core running on
real FPGA hardware, which computes the actual frame-to-frame motion
(subtraction) per screen region.

Each frame is split into a 2x2 grid -- top-left, top-right, bottom-left,
bottom-right -- exactly matching the chip's 4-lane SIMD width. Average
brightness per region becomes one real-world 4-element vector per frame.
The chip computes (this_frame - previous_frame) per region on real
silicon, giving a real motion-magnitude reading per quadrant.

No extra hardware needed -- just the webcam already in your laptop and
the FPGA + UART link already verified working.

Usage:
    pip install --break-system-packages opencv-python
    python3 webcam_motion.py /dev/ttyUSB0
Press 'q' in the video window (or Ctrl+C in the terminal) to quit.
"""

import sys
import time
import struct
import argparse
import serial

try:
    import cv2
except ImportError:
    print("This demo needs OpenCV. Install it with:")
    print("  pip install --break-system-packages opencv-python")
    sys.exit(1)

OP_LOAD, OP_SUB = 0, 2


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


def send_and_receive(ser, cmd, retries=1):
    ser.write(cmd)
    ser.flush()
    resp = ser.read(9)
    if len(resp) != 9:
        if retries > 0:
            return send_and_receive(ser, cmd, retries - 1)
        raise RuntimeError(f"expected 9 bytes, got {len(resp)}")
    return decode_response(resp)


def quantize_block(values):
    """Quantize a 4-element real-world vector into the chip's native
    shared-exponent block format. Brightness values are always in
    0-255 range (or small differences), so a small fixed exponent works
    well and keeps things fast -- no need to search for a scale each frame."""
    exp = -6  # gives resolution of 1/64 per mantissa unit, plenty for 0-255 brightness
    scale = 2.0 ** (-exp)
    mants = [max(-32768, min(32767, round(v * scale))) for v in values]
    return mants, exp


def region_brightness(gray_frame):
    h, w = gray_frame.shape
    h2, w2 = h // 2, w // 2
    tl = gray_frame[0:h2, 0:w2].mean()
    tr = gray_frame[0:h2, w2:w].mean()
    bl = gray_frame[h2:h, 0:w2].mean()
    br = gray_frame[h2:h, w2:w].mean()
    return [float(tl), float(tr), float(bl), float(br)]


def bar(value, width=20, scale=10.0):
    n = min(width, int(abs(value) / scale * width))
    return "#" * n + "." * (width - n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    print(f"Connecting to bfp_simd_core on {args.port}...")
    ser = serial.Serial(args.port, 9600, timeout=5,
                         rtscts=False, dsrdtr=False, xonxoff=False)
    ser.setRTS(True)
    ser.setDTR(True)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Could not open camera {args.camera}")
        sys.exit(1)

    prev_vals = None
    print("Running. Press 'q' in the video window to quit.\n")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vals = region_brightness(gray)

            if prev_vals is not None:
                mants_a, exp_a = quantize_block(vals)
                mants_b, exp_b = quantize_block(prev_vals)

                cmd = build_command(OP_LOAD, waddr=0, raddr1=0, raddr2=0,
                                     lane_enable=0b1111, imm_exp=exp_a, imm_mant=mants_a)
                send_and_receive(ser, cmd)
                cmd = build_command(OP_LOAD, waddr=1, raddr1=0, raddr2=0,
                                     lane_enable=0b1111, imm_exp=exp_b, imm_mant=mants_b)
                send_and_receive(ser, cmd)
                cmd = build_command(OP_SUB, waddr=2, raddr1=0, raddr2=1,
                                     lane_enable=0b1111, imm_exp=0, imm_mant=[0, 0, 0, 0])
                mants, exp, motion = send_and_receive(ser, cmd)

                print(
                    f"\rTL {bar(motion[0])} {motion[0]:+6.2f}  "
                    f"TR {bar(motion[1])} {motion[1]:+6.2f}  "
                    f"BL {bar(motion[2])} {motion[2]:+6.2f}  "
                    f"BR {bar(motion[3])} {motion[3]:+6.2f}   ",
                    end="", flush=True,
                )

                # overlay motion readout on the video window too
                cv2.putText(frame, f"HW motion: {[round(m,1) for m in motion]}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            prev_vals = vals
            cv2.imshow("webcam (press q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        cap.release()
        cv2.destroyAllWindows()
        ser.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
