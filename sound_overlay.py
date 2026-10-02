#!/usr/bin/env python3
"""
sound_overlay.py - draw the array's bearing on a live webcam picture.

    python sound_overlay.py --port COM4 --hfov 62

The board prints, once per report interval, either

    ANGLE az=12.40 el=-3.10      a sound, measured
    ANGLE none                   quiet, or too weak to trust

This reads those lines and paints a marker where the sound came from.
Nothing here talks to the microphones: the webcam is only a backdrop,
and the serial port is the whole connection between the two.

Keys (with the video window focused)
    q / Esc   quit
    m         mirror the picture (selfie view)
    t         trail on / off
    c         clear the trail
    s         save a PNG
    f         freeze / unfreeze
"""

import argparse
import math
import queue
import re
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

try:
    import serial                      # pyserial
except ImportError:
    serial = None


# ---------------------------------------------------------------- serial ---

class AngleReader(threading.Thread):
    """
    Reads 'ANGLE az=.. el=..' and 'ANGLE none' from the board.

    Anchored on the labels, never on position: the board also prints
    human-readable lines that begin with counters and lag values, and a
    'first two numbers on the line' parser would read those by mistake.
    """

    RX_PAIR = re.compile(
        r"\bangle\b.*?\baz\s*=\s*([-+]?\d+(?:\.\d+)?)"
        r".*?\bel\s*=\s*([-+]?\d+(?:\.\d+)?)",
        re.I)
    RX_NONE = re.compile(r"\bangle\s+none\b", re.I)
    RX_Q = re.compile(r"\bq\s+(\d+(?:\.\d+)?)", re.I)

    def __init__(self, port, baud=115200):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = queue.Queue(maxsize=64)
        self.status = "opening"
        self.last_raw = ""
        self.quality = None
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        if serial is None:
            self.status = "pyserial not installed"
            return
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.3)
        except Exception as exc:                      # port busy, wrong name
            self.status = f"{type(exc).__name__}: {exc}"
            return

        self.status = "connected"
        with ser:
            while not self._stop.is_set():
                try:
                    raw = ser.readline()
                except Exception as exc:
                    self.status = f"read failed: {exc}"
                    return
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                self.last_raw = line

                m = self.RX_Q.search(line)
                if m:
                    self.quality = float(m.group(1))

                if self.RX_NONE.search(line):
                    self._put(None)
                    continue

                m = self.RX_PAIR.search(line)
                if m:
                    self._put((float(m.group(1)), float(m.group(2))))

    def _put(self, item):
        try:
            self.q.put_nowait((item, time.time()))
        except queue.Full:
            pass


# ------------------------------------------------------------- geometry ---

def angle_to_pixel(az_deg, el_deg, w, h, hfov_deg, mirror):
    """
    Map a bearing onto the image using a pinhole model.

    A linear degrees-to-pixels map is only right near the centre; tan()
    keeps the marker on the object all the way to the frame edge.
    """
    half_h = math.radians(hfov_deg) / 2.0
    tan_h = math.tan(half_h)
    tan_v = tan_h * (h / float(w))              # same focal length both axes

    x = (w / 2.0) * (1.0 + math.tan(math.radians(az_deg)) / tan_h)
    y = (h / 2.0) * (1.0 - math.tan(math.radians(el_deg)) / tan_v)

    if mirror:
        x = w - x
    return x, y


# --------------------------------------------------------------- drawing ---

def draw_marker(img, x, y, age, quality):
    """Crosshair plus a ring that shrinks as the reading gets older."""
    h, w = img.shape[:2]
    fade = max(0.0, 1.0 - age / 1.2)            # gone after 1.2 s
    if fade <= 0.0:
        return

    xi, yi = int(round(x)), int(round(y))
    r = int(18 + 26 * (1.0 - fade))

    # colour by quality: amber when marginal, green when confident
    if quality is None:
        col = (80, 220, 80)
    elif quality >= 3.0:
        col = (80, 220, 80)
    elif quality >= 2.0:
        col = (60, 200, 235)
    else:
        col = (70, 140, 250)

    col = tuple(int(c * (0.35 + 0.65 * fade)) for c in col)
    thick = 2 if fade > 0.4 else 1

    cv2.circle(img, (xi, yi), r, col, thick, cv2.LINE_AA)
    gap = r // 2
    cv2.line(img, (xi - r - 10, yi), (xi - gap, yi), col, thick, cv2.LINE_AA)
    cv2.line(img, (xi + gap, yi), (xi + r + 10, yi), col, thick, cv2.LINE_AA)
    cv2.line(img, (xi, yi - r - 10), (xi, yi - gap), col, thick, cv2.LINE_AA)
    cv2.line(img, (xi, yi + gap), (xi, yi + r + 10), col, thick, cv2.LINE_AA)

    # off-frame: clamp to the border and point outward
    if not (0 <= xi < w and 0 <= yi < h):
        cx, cy = min(max(xi, 12), w - 12), min(max(yi, 12), h - 12)
        cv2.circle(img, (cx, cy), 9, col, -1, cv2.LINE_AA)


def draw_grid(img, hfov, mirror):
    """Faint reference lines every 15 degrees, so you can read the scale."""
    h, w = img.shape[:2]
    col = (60, 60, 60)
    for deg in range(-75, 76, 15):
        x, _ = angle_to_pixel(deg, 0, w, h, hfov, mirror)
        if 0 <= x < w:
            cv2.line(img, (int(x), 0), (int(x), h), col, 1, cv2.LINE_AA)
        _, y = angle_to_pixel(0, deg, w, h, hfov, mirror)
        if 0 <= y < h:
            cv2.line(img, (0, int(y)), (w, int(y)), col, 1, cv2.LINE_AA)

    cx, cy = angle_to_pixel(0, 0, w, h, hfov, mirror)
    cv2.drawMarker(img, (int(cx), int(cy)), (110, 110, 110),
                   cv2.MARKER_CROSS, 18, 1, cv2.LINE_AA)


def draw_hud(img, lines):
    pad, lh = 10, 20
    box_h = pad * 2 + lh * len(lines)
    strip = img[0:box_h, 0:330]
    img[0:box_h, 0:330] = (strip * 0.35).astype(np.uint8)
    for i, (text, col) in enumerate(lines):
        cv2.putText(img, text, (pad, pad + lh * (i + 1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1, cv2.LINE_AA)


# ------------------------------------------------------------------ main ---

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="serial port, e.g. COM4 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cam", type=int, default=0, help="camera index")
    ap.add_argument("--hfov", type=float, default=62.0,
                    help="camera horizontal field of view, degrees")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--trail", type=int, default=12,
                    help="how many past readings to keep on screen")
    ap.add_argument("--mirror", action="store_true",
                    help="start in selfie view")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW if sys.platform == "win32"
                           else cv2.CAP_ANY)
    if not cap.isOpened():
        print(f"cannot open camera {args.cam}", file=sys.stderr)
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    reader = None
    if args.port:
        reader = AngleReader(args.port, args.baud)
        reader.start()
    else:
        print("no --port given: the picture will run with no bearings")

    trail = deque(maxlen=max(1, args.trail))
    current = None              # (az, el, t) or None
    mirror = args.mirror
    show_trail = True
    frozen = False
    frame = None
    t_fps, fps = time.time(), 0.0
    saved = 0

    win = "acoustic camera"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        if not frozen or frame is None:
            ok, grabbed = cap.read()
            if not ok:
                print("camera read failed", file=sys.stderr)
                break
            frame = grabbed

        img = frame.copy()
        if mirror:
            img = cv2.flip(img, 1)
        h, w = img.shape[:2]

        # ---- drain whatever the board has said since the last frame ----
        if reader is not None:
            while True:
                try:
                    item, stamp = reader.q.get_nowait()
                except queue.Empty:
                    break
                if item is None:
                    current = None          # explicit "nothing to report"
                else:
                    az, el = item
                    current = (az, el, stamp)
                    trail.append((az, el, stamp))

        draw_grid(img, args.hfov, mirror)

        now = time.time()
        if show_trail:
            for az, el, stamp in list(trail):
                age = now - stamp
                if age > 1.2:
                    continue
                x, y = angle_to_pixel(az, el, w, h, args.hfov, mirror)
                draw_marker(img, x, y, age,
                            reader.quality if reader else None)

        if current is not None:
            az, el, stamp = current
            x, y = angle_to_pixel(az, el, w, h, args.hfov, mirror)
            draw_marker(img, x, y, 0.0, reader.quality if reader else None)
            cv2.putText(img, f"{az:+.1f}, {el:+.1f}",
                        (int(x) + 26, int(y) - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 2,
                        cv2.LINE_AA)

        # ---- HUD --------------------------------------------------------
        fps = 0.9 * fps + 0.1 / max(1e-3, now - t_fps)
        t_fps = now

        if reader is None:
            link = ("no serial port", (120, 120, 250))
        elif reader.status != "connected":
            link = (f"serial: {reader.status}", (120, 120, 250))
        elif current is None:
            link = ("listening - no sound", (170, 170, 170))
        else:
            link = ("SOUND", (80, 230, 80))

        hud = [link]
        if current is not None:
            az, el, _ = current
            qtxt = f"  q {reader.quality:.2f}" if (reader and reader.quality) else ""
            hud.append((f"az {az:+6.1f}   el {el:+6.1f}{qtxt}", (230, 230, 230)))
        else:
            hud.append(("az   --      el   --", (150, 150, 150)))
        hud.append((f"hfov {args.hfov:.0f}   {w}x{h}   {fps:4.1f} fps"
                    f"{'   MIRROR' if mirror else ''}"
                    f"{'   FROZEN' if frozen else ''}", (150, 150, 150)))
        draw_hud(img, hud)

        cv2.imshow(win, img)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        elif k == ord('m'):
            mirror = not mirror
        elif k == ord('t'):
            show_trail = not show_trail
        elif k == ord('c'):
            trail.clear()
            current = None
        elif k == ord('f'):
            frozen = not frozen
        elif k == ord('s'):
            saved += 1
            name = f"acoustic_{int(time.time())}_{saved}.png"
            cv2.imwrite(name, img)
            print("saved", name)

    if reader is not None:
        reader.stop()
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
