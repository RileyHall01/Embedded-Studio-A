#!/usr/bin/env python3
"""
health_monitor_gui.py
PC display for the Group 4 FPGA health monitor (48451 Embedded Systems Studio A).

The FPGA sends one packet per line to the Arduino, which forwards it to the
PC over USB at 115200 baud:

    $HR=000,T=24.6,H=023,M=0,X=+00032,Y=+00000,Z=+00243,E=0000,F=01*1F

    HR      heart rate in BPM (0 = no valid reading)
    T       temperature in deg C
    H       relative humidity in %
    M       movement flag (1 = moving, 0 = stationary)
    X Y Z   acceleration in ADXL345 counts (256 counts = 1 g)
    E       ECG value, plotted as the ECG trace
    F       fault/status byte, two hex digits
    *CS     XOR of every character between '$' and '*', two hex digits

Packets that fail the checksum are counted and discarded. If no valid packet
arrives for LINK_TIMEOUT_S, the display shows that the link is lost.

Libraries used:

    PySerial
    Matplotlib
    Tkinter

"""

import argparse
import collections
import queue
import re
import threading
import time
import tkinter as tk
import tkinter.font as tkfont

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial
import serial.tools.list_ports

# ----------------------------------------------------------------- settings
BAUD = 115200            # Arduino -> PC link (the FPGA -> Arduino link is 9600)
COUNTS_PER_G = 256.0     # ADXL345 at full resolution, +/-2 g
PLOT_SECONDS = 10.0      # width of the scrolling plots
LINK_TIMEOUT_S = 1.0     # spec 11: data must be shown within 1 s
REFRESH_MS = 100         # display refresh period

# Names for the fault byte bits. Fill these in to match the FPGA packet
# builder. Bits without a name are shown as "bit n".
FAULT_BIT_NAMES = {
    # 0: "name of bit 0",
}

# Colours
BG, CARD, BORDER = "#f3f4f6", "#ffffff", "#d1d5db"
TEXT, MUTED = "#111827", "#6b7280"
GOOD, WARN, BAD = "#15803d", "#b45309", "#b91c1c"

# ----------------------------------------------------------------- parsing
PACKET_RE = re.compile(r"^\$([^$*]*)\*([0-9A-Fa-f]{2})$")
LEGACY_RE = re.compile(r"^(-?\d+),(-?\d+),(-?\d+),(\d+)$")    # old accel-only "x,y,z,m" line
RATE_RE = re.compile(r"Sample Rate:\s*[-0-9.]+\s*Hz")          # Arduino's once-a-second message


def xor_checksum(text):
    """XOR of every character, matching the FPGA packet builder."""
    cs = 0
    for ch in text:
        cs ^= ord(ch)
    return cs


def _field(fields, key, conv):
    """Convert one packet field, or return None if it is missing or malformed."""
    try:
        return conv(fields[key])
    except (KeyError, ValueError):
        return None


def parse_line(line):
    """Return a dict of readings for a valid line, "bad" for a corrupted
    packet, or None for any other text (e.g. the Arduino's start-up message)."""
    m = PACKET_RE.match(line)
    if m:
        body, cs = m.groups()
        if xor_checksum(body) != int(cs, 16):
            return "bad"
        fields = dict(item.split("=", 1) for item in body.split(",") if "=" in item)
        return {
            "hr": _field(fields, "HR", int),
            "t":  _field(fields, "T", float),
            "h":  _field(fields, "H", int),
            "m":  _field(fields, "M", int),
            "x":  _field(fields, "X", int),
            "y":  _field(fields, "Y", int),
            "z":  _field(fields, "Z", int),
            "e":  _field(fields, "E", int),
            "f":  _field(fields, "F", lambda v: int(v, 16)),
        }
    if line.startswith("$"):
        return "bad"
    m = LEGACY_RE.match(line)
    if m:
        x, y, z, mv = (int(g) for g in m.groups())
        return {"x": x, "y": y, "z": z, "m": 1 if mv else 0}
    return None


class LineAssembler:
    """Rejoins packets split by the Arduino's "Sample Rate" message.

    The Arduino prints its packet rate once a second between forwarded bytes,
    so the message can land inside an FPGA packet and split it over two lines.
    The message is removed and the two halves are joined again."""

    def __init__(self):
        self.pending = ""

    def feed(self, raw):
        line = raw.strip()
        cut = False
        m = RATE_RE.search(line)
        if m:
            line = (line[:m.start()] + line[m.end():]).strip()
            cut = True
        if self.pending:
            if not line.startswith("$"):        # second half of the split packet
                line = self.pending + line
            self.pending = ""
        if cut and line.startswith("$") and not PACKET_RE.match(line):
            self.pending = line                  # first half: wait for the rest
            return None
        return line or None


# ----------------------------------------------------------- serial reader
def find_arduino_port():
    """Return the most likely Arduino serial port, or None."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:                              # Arduino, Arduino.org, CH340 clones
        if p.vid in (0x2341, 0x2A03, 0x1A86):
            return p.device
    for p in ports:
        if any(s in p.device for s in ("ttyACM", "ttyUSB", "usbmodem")):
            return p.device
    return None


class SerialReader(threading.Thread):
    """Reads the serial port in the background and posts results to a queue.
    Reconnects automatically if the port is missing or unplugged."""

    def __init__(self, port, baud, out_q):
        super().__init__(daemon=True)
        self.port, self.baud, self.out_q = port, baud, out_q
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            port = self.port or find_arduino_port()
            if port is None:
                self.out_q.put(("status", "No serial port found", None))
                time.sleep(1.0)
                continue
            try:
                with serial.Serial(port, self.baud, timeout=0.1) as ser:
                    self.out_q.put(("status", f"Connected to {port}", port))
                    self._read_loop(ser)
            except (serial.SerialException, OSError):
                self.out_q.put(("status", f"Waiting for {port}", None))
                time.sleep(1.0)

    def _read_loop(self, ser):
        assembler = LineAssembler()
        buf = b""
        while not self.stop_event.is_set():
            buf += ser.read(ser.in_waiting or 1)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = assembler.feed(raw.decode("ascii", "replace"))
                if line is None:
                    continue
                result = parse_line(line)
                if result == "bad":
                    self.out_q.put(("bad", line, None))
                elif result is not None:
                    self.out_q.put(("data", result, line))
            if len(buf) > 1024:                  # no line ending for too long: resync
                buf = b""


# ----------------------------------------------------------------- display
class HealthMonitorApp:
    def __init__(self, root, reader):
        self.root, self.reader, self.q = root, reader, reader.out_q
        self.latest = {}                         # most recent value of each field
        self.raw_line = ""
        self.last_rx = None                      # time of the last valid packet
        self.rx_times = collections.deque()      # packet times, for packets/s
        self.bad_count = 0
        self.conn_text, self.connected = "Starting", False
        self.history = collections.deque(maxlen=int(PLOT_SECONDS * 50))  # (t, e, x, y, z)

        family = tkfont.nametofont("TkDefaultFont").actual()["family"]
        self.f_small, self.f_value = (family, 10), (family, 26, "bold")
        self.f_word = (family, 20, "bold")
        self.f_title = (family, 16, "bold")

        root.title("Group 4 Health Monitor")
        root.configure(bg=BG)
        root.geometry("1100x820")
        root.minsize(900, 680)
        self._build()
        root.protocol("WM_DELETE_WINDOW", self._close)
        root.after(REFRESH_MS, self._tick)

    # ---- layout
    def _card(self, parent, row, col, title):
        frame = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER)
        frame.grid(row=row, column=col, sticky="nsew", padx=6, pady=6)
        tk.Label(frame, text=title, bg=CARD, fg=MUTED, font=self.f_small).pack(anchor="w", padx=12, pady=(8, 0))
        value = tk.Label(frame, text="--", bg=CARD, fg=TEXT, font=self.f_value)
        value.pack(anchor="w", padx=12)
        detail = tk.Label(frame, text="", bg=CARD, fg=MUTED, font=self.f_small)
        detail.pack(anchor="w", padx=12, pady=(0, 8))
        return value, detail

    def _build(self):
        head = tk.Frame(self.root, bg=BG)
        head.pack(fill="x", padx=12, pady=(10, 0))
        tk.Label(head, text="Group 4 Health Monitor", bg=BG, fg=TEXT, font=self.f_title).pack(side="left")
        self.link_lbl = tk.Label(head, text="NO DATA", bg=BAD, fg="white",
                                 font=(self.f_small[0], 11, "bold"), padx=10, pady=3)
        self.link_lbl.pack(side="right")

        grid = tk.Frame(self.root, bg=BG)
        grid.pack(fill="x", padx=6, pady=4)
        for c in range(4):
            grid.columnconfigure(c, weight=1, uniform="card")
        self.c_hr = self._card(grid, 0, 0, "Heart rate")
        self.c_t = self._card(grid, 0, 1, "Temperature")
        self.c_h = self._card(grid, 0, 2, "Humidity")
        self.c_m = self._card(grid, 0, 3, "Movement")
        self.c_x = self._card(grid, 1, 0, "Acceleration X")
        self.c_y = self._card(grid, 1, 1, "Acceleration Y")
        self.c_z = self._card(grid, 1, 2, "Acceleration Z")
        self.c_f = self._card(grid, 1, 3, "Fault status")
        for card in (self.c_m, self.c_f):        # word values need a smaller font
            card[0].config(font=self.f_word)

        self.fig = Figure(figsize=(10, 4.2), dpi=100, facecolor=BG)
        self.ax_ecg = self.fig.add_subplot(2, 1, 1)
        self.ax_acc = self.fig.add_subplot(2, 1, 2, sharex=self.ax_ecg)
        self.ln_ecg, = self.ax_ecg.plot([], [], color="#b91c1c", lw=1.4)
        self.ln_x, = self.ax_acc.plot([], [], color="#2563eb", lw=1.2, label="X")
        self.ln_y, = self.ax_acc.plot([], [], color="#d97706", lw=1.2, label="Y")
        self.ln_z, = self.ax_acc.plot([], [], color="#059669", lw=1.2, label="Z")
        self.ax_ecg.set_ylabel("ECG (E)")
        self.ax_acc.set_ylabel("Acceleration (g)")
        self.ax_acc.set_xlabel("Time (s)")
        self.ax_acc.set_ylim(-2.1, 2.1)          # ADXL345 range is +/-2 g
        self.ax_acc.legend(loc="upper left", ncol=3, fontsize=8, frameon=False)
        for ax in (self.ax_ecg, self.ax_acc):
            ax.set_xlim(-PLOT_SECONDS, 0)
            ax.grid(True, color="#e5e7eb")
            ax.set_facecolor(CARD)
        self.fig.tight_layout()
        # status lines are packed before the plot so the plot cannot push them off screen
        self.raw_lbl = tk.Label(self.root, text="", bg=BG, fg=TEXT, font="TkFixedFont", anchor="w")
        self.raw_lbl.pack(side="bottom", fill="x", padx=12, pady=(0, 10))
        self.status_lbl = tk.Label(self.root, text="", bg=BG, fg=MUTED, font=self.f_small, anchor="w")
        self.status_lbl.pack(side="bottom", fill="x", padx=12)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=4)

    # ---- updates
    def _drain_queue(self):
        now = time.monotonic()
        for _ in range(500):
            try:
                kind, payload, extra = self.q.get_nowait()
            except queue.Empty:
                break
            if kind == "status":
                self.conn_text, self.connected = payload, extra is not None
            elif kind == "bad":
                self.bad_count += 1
            elif kind == "data":
                self.latest.update({k: v for k, v in payload.items() if v is not None})
                self.raw_line, self.last_rx = extra, now
                self.rx_times.append(now)
                nan = float("nan")
                g = lambda k: payload[k] / COUNTS_PER_G if payload.get(k) is not None else nan
                e = payload.get("e")
                self.history.append((now, nan if e is None else e, g("x"), g("y"), g("z")))

    def _set(self, card, value, detail, colour, stale):
        card[0].config(text=value, fg=MUTED if stale else colour)
        card[1].config(text=detail)

    def _update_cards(self, stale):
        d = self.latest
        hr = d.get("hr")
        if hr:
            self._set(self.c_hr, f"{hr}", "BPM", TEXT, stale)
        else:
            self._set(self.c_hr, "--", "BPM (no valid reading)" if hr == 0 else "BPM", TEXT, stale)

        t, h = d.get("t"), d.get("h")
        self._set(self.c_t, "--" if t is None else f"{t:.1f}", "°C", TEXT, stale)
        self._set(self.c_h, "--" if h is None else f"{h}", "% relative humidity", TEXT, stale)

        m = d.get("m")
        if m is None:
            self._set(self.c_m, "--", "", TEXT, stale)
        elif m == 1:
            self._set(self.c_m, "MOVING", "M=1", WARN, stale)
        elif m == 0:
            self._set(self.c_m, "STATIONARY", "M=0", GOOD, stale)
        else:
            self._set(self.c_m, f"M={m}", "", TEXT, stale)

        for card, key in ((self.c_x, "x"), (self.c_y, "y"), (self.c_z, "z")):
            raw = d.get(key)
            if raw is None:
                self._set(card, "--", "g", TEXT, stale)
            else:
                self._set(card, f"{raw / COUNTS_PER_G:+.2f}", f"g   (raw {raw:+d})", TEXT, stale)

        f = d.get("f")
        if f is None:
            self._set(self.c_f, "--", "", TEXT, stale)
        elif f == 0:
            self._set(self.c_f, "OK", "F=00", GOOD, stale)
        else:
            bits = [FAULT_BIT_NAMES.get(b, f"bit {b}") for b in range(8) if f >> b & 1]
            self._set(self.c_f, "FAULT", f"F={f:02X}: " + ", ".join(bits), BAD, stale)

    def _update_status(self, now, stale):
        while self.rx_times and now - self.rx_times[0] > 2.0:
            self.rx_times.popleft()
        rate = len(self.rx_times) / 2.0
        if self.last_rx is None:
            self.link_lbl.config(text="NO DATA", bg=BAD)
            age = "none yet"
        else:
            self.link_lbl.config(text="LINK LOST" if stale else "LIVE", bg=BAD if stale else GOOD)
            age = f"{now - self.last_rx:.1f} s ago"
        self.status_lbl.config(text=f"{self.conn_text}   |   {rate:.1f} packets/s   |   "
                                    f"bad packets: {self.bad_count}   |   last packet: {age}")
        self.raw_lbl.config(text=f"Last packet:  {self.raw_line}" if self.raw_line else "")

    def _update_plots(self, now):
        if not self.history:
            return
        t = [p[0] - now for p in self.history]
        self.ln_ecg.set_data(t, [p[1] for p in self.history])
        self.ln_x.set_data(t, [p[2] for p in self.history])
        self.ln_y.set_data(t, [p[3] for p in self.history])
        self.ln_z.set_data(t, [p[4] for p in self.history])
        self.ax_ecg.relim()
        self.ax_ecg.autoscale_view(scalex=False, scaley=True)
        self.canvas.draw_idle()

    def _tick(self):
        self._drain_queue()
        now = time.monotonic()
        stale = self.last_rx is None or now - self.last_rx > LINK_TIMEOUT_S
        self._update_cards(stale)
        self._update_status(now, stale)
        self._update_plots(now)
        self.root.after(REFRESH_MS, self._tick)

    def _close(self):
        self.reader.stop_event.set()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="Group 4 FPGA health monitor display")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0 or COM5 (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=BAUD, help=f"baud rate (default {BAUD})")
    args = ap.parse_args()

    reader = SerialReader(args.port, args.baud, queue.Queue())
    root = tk.Tk()
    HealthMonitorApp(root, reader)
    reader.start()
    root.mainloop()


if __name__ == "__main__":
    main()
