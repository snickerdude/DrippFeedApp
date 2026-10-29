#!/usr/bin/env python3
"""
GCODE Drip Feeder
-----------------
Watches a folder; when a GCODE file is added/modified it is streamed to a CNC
over a USB-serial adapter with selectable flow control.

Requires:  pip install pyserial        (tkinter ships with most Python installs)
"""

"""
TODO:
send file now button should send the selected file if a specific file is selected.
add presets for serial parameters, store in the json
done - custom icon
"""

import json
import queue
import re
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
import ctypes
import os

import serial
from serial.tools import list_ports

CONFIG_PATH = Path(os.getcwd() + "\\.gcode_feeder.json")
GCODE_EXTS = {".gcode", ".nc", ".ngc", ".tap", ".gc", ".cnc", ".txt"}
ANY_FILE = "<any file in folder>"

# (VID, PID) -> friendly name, so we can tell which adapter is which
KNOWN_ADAPTERS = {
    (0x0403, 0x6001): "FTDI FT232R",
    (0x0403, 0x6010): "FTDI FT2232",
    (0x0403, 0x6011): "FTDI FT4232",
    (0x0403, 0x6014): "FTDI FT232H",
    (0x0403, 0x6015): "FTDI FT230X",
    (0x067B, 0x2303): "Prolific PL2303",
    (0x067B, 0x23A3): "Prolific PL2303GC",
    (0x067B, 0x23C3): "Prolific PL2303GT",
}

PARITY = {"None": serial.PARITY_NONE, "Even": serial.PARITY_EVEN,
          "Odd": serial.PARITY_ODD}
STOPBITS = {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE,
            "2": serial.STOPBITS_TWO}
FLOW = ["None", "RTS/CTS", "XON/XOFF", "XON/XOFF (app-level)", "DSR/DTR"]
APP_XON = "XON/XOFF (app-level)"
XON, XOFF = 0x11, 0x13
HANDSHAKE = ["None (flow control only)", "Wait for 'ok' per line"]
EOLS = {"LF (\\n)": "\n", "CR (\\r)": "\r", "CRLF (\\r\\n)": "\r\n"}
BAUDS = ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200",
         "230400", "250000"]

# Settings saved in / restored from a preset (the port is stored as e.g. "COM5")
PRESET_KEYS = ("port", "baud", "databits", "parity", "stopbits", "flow",
               "handshake", "eol", "line_delay_ms", "ack_timeout", "chunk")
# Seeded into the json the first time; edit/delete freely from the GUI
DEFAULT_PRESETS = {
    "Fanuc 0i": {"port": "", "baud": "9600", "databits": "7", "parity": "Even",
                 "stopbits": "2", "flow": APP_XON, "handshake": HANDSHAKE[0],
                 "eol": "LF (\\n)", "line_delay_ms": "0", "ack_timeout": "10",
                 "chunk": "4", "strip_comments": False, "fanuc_percent": True},
}

def fmt_bytes_pair(done, total):
    """'12.3/45.7 KB' - one unit (chosen from the total) keeps the text short."""
    if total < 1024:
        div, unit, dec = 1, "B", 0
    elif total < 1024 ** 2:
        div, unit, dec = 1024, "KB", 1
    else:
        div, unit, dec = 1024 ** 2, "MB", 2
    return f"{done / div:.{dec}f}/{total / div:.{dec}f} {unit}"
# --------------------------------------------------------------------------
# Serial port discovery
# --------------------------------------------------------------------------
def list_serial_ports():
    """Return [(device, description)] with adapter identification."""
    out = []
    for p in list_ports.comports():
        kind = KNOWN_ADAPTERS.get((p.vid, p.pid)) if p.vid else None
        desc = kind or p.description or "unknown"
        if p.vid:
            desc += f"  [{p.vid:04X}:{p.pid:04X}]"
        out.append((p.device, desc))
    return out


# --------------------------------------------------------------------------
# Directory watcher (polling: no extra dependencies, works on every OS)
# --------------------------------------------------------------------------
class Watcher(threading.Thread):
    def __init__(self, directory, target_name, ui_q, interval=1.0):
        super().__init__(daemon=True)
        self.dir = Path(directory)
        self.target = target_name  # None => any gcode file
        self.q = ui_q
        self.interval = interval
        self.stop_evt = threading.Event()

    def _snapshot(self):
        snap = {}
        try:
            for f in self.dir.iterdir():
                if not f.is_file():
                    continue
                if self.target:
                    if f.name != self.target:
                        continue
                elif f.suffix.lower() not in GCODE_EXTS:
                    continue
                st = f.stat()
                snap[f] = (st.st_mtime, st.st_size)
        except OSError:
            pass
        return snap

    def run(self):
        known = self._snapshot()  # existing files do NOT trigger on enable
        while not self.stop_evt.wait(self.interval):
            now = self._snapshot()
            for f, sig in now.items():
                if known.get(f) != sig and self._settled(f, sig):
                    self.q.put(("file", str(f)))
                    now[f] = f.stat().st_mtime, f.stat().st_size
            known = now

    def _settled(self, f, sig):
        """Wait until size stops changing (file finished being written)."""
        last = sig
        for _ in range(10):
            if self.stop_evt.wait(0.5):
                return False
            try:
                st = f.stat()
            except OSError:
                return False
            cur = (st.st_mtime, st.st_size)
            if cur == last:
                return True
            last = cur
        return True


# --------------------------------------------------------------------------
# Sender thread
# --------------------------------------------------------------------------
class Sender(threading.Thread):
    def __init__(self, path, cfg, ui_q):
        super().__init__(daemon=True)
        self.path, self.cfg, self.q = path, cfg, ui_q
        self.stop_evt = threading.Event()
        self.go_evt = threading.Event()
        self.go_evt.set()  # cleared => paused
        self.app_xon = cfg["flow"] == APP_XON
        self.xoff = True  # True while the machine has told us to stop

    def pause(self, paused):
        self.go_evt.clear() if paused else self.go_evt.set()

    def stop(self):
        self.stop_evt.set()
        self.go_evt.set()

    def log(self, msg):
        self.q.put(("log", msg))

    def _prepare(self):
        lines = []
        text = Path(self.path).read_text(errors="replace")
        for raw in text.splitlines():
            line = raw
            if self.cfg["strip_comments"]:
                line = re.sub(r"\(.*?\)", "", line)  # (comments) only;
                # ';' is the Fanuc end-of-block char, never a comment
            line = line.strip()
            if line:
                lines.append(line)
        if self.cfg["fanuc_percent"] and lines:
            if lines[0] != "%":
                lines.insert(0, "%")
            if lines[-1] != "%":
                lines.append("%")
        return lines

    def _open(self):
        c = self.cfg
        flow = c["flow"]
        return serial.Serial(
            port=c["port"],
            baudrate=int(c["baud"]),
            bytesize=int(c["databits"]),
            parity=PARITY[c["parity"]],
            stopbits=STOPBITS[c["stopbits"]],
            xonxoff=(flow == "XON/XOFF"),
            rtscts=(flow == "RTS/CTS"),
            dsrdtr=(flow == "DSR/DTR"),
            timeout=0.1,
            write_timeout=None,  # block while CTS/XOFF holds us off
        )

    def _poll_rx(self, ser):
        """Read whatever the machine sent; track XON/XOFF; log the rest."""
        n = ser.in_waiting
        if not n:
            return
        data = ser.read(n)
        
        if self.app_xon:
            last = max(data.rfind(bytes([XON])), data.rfind(bytes([XOFF])))
            if last >= 0:
                self.xoff = data[last] == XOFF
                #self.log("XOFF status: " + str(self.xoff))

    def _send(self, ser, payload):
        """Send one block without ever blocking indefinitely. False => stop."""
        delay = float(self.cfg["line_delay_ms"]) / 1000.0
        if self.app_xon: #not self.xoff and
            chunk = max(1, int(self.cfg["chunk"]))
            for i in range(0, len(payload), chunk):
                
                while not self.stop_evt.is_set():
                    self.go_evt.wait()
                    if delay:
                        time.sleep(delay)
                    self._poll_rx(ser)
                    if not self.xoff and ser.out_waiting == 0:
                        break
                    if self.stop_evt.is_set():
                        return False
                ser.write(payload[i:i+ chunk])
            return True
        while ser.out_waiting > 128:  # driver/hardware flow control holding us
            if self.stop_evt.is_set():
                return False
            self._poll_rx(ser)
            time.sleep(0.01)
        ser.write(payload)
        return True

    def _drain(self, ser, timeout=20):
        end = time.time() + timeout
        while ser.out_waiting or (self.app_xon and self.xoff):
            if self.stop_evt.is_set() or time.time() > end:
                self.log("Machine still holding off at end of file")
                return
            self._poll_rx(ser)
            time.sleep(0.01)

    def run(self):
        try:
            self._run()
        except Exception as e:  # noqa: BLE001
            self.log(f"ERROR: {e}")
        finally:
            self.q.put(("done", None))

    def _wait_ack(self, ser):
        """Return True on 'ok', False on error/stop."""
        deadline = time.time() + float(self.cfg["ack_timeout"])
        while not self.stop_evt.is_set():
            resp = ser.readline().decode(errors="replace").strip()
            if resp:
                low = resp.lower()
                if low.startswith("ok"):
                    return True
                self.log(f"<< {resp}")
                if low.startswith("error") or "alarm" in low:
                    return False
            elif time.time() > deadline:
                self.log("Timed out waiting for 'ok'")
                return False
        return False

    def _run(self):
        lines = self._prepare()
        total = len(lines)
        self.log(f"Loaded {total} lines from {Path(self.path).name}")
        eol = EOLS[self.cfg["eol"]].encode()
        use_ack = self.cfg["handshake"] == HANDSHAKE[1]
        delay = float(self.cfg["line_delay_ms"]) / 1000.0
        total_bytes = sum(len(l.encode("ascii", "ignore")) + len(eol) for l in lines)
        sent = 0

        with self._open() as ser:
            self.log(f"Opened {ser.port} @ {ser.baudrate} "
                     f"({self.cfg['flow']} flow control)")
            ser.reset_input_buffer()
            for i, line in enumerate(lines, 1):
                self.go_evt.wait()
                if self.stop_evt.is_set():
                    self.log("Stopped by user")
                    return
                
                if use_ack:
                    if not self._wait_ack(ser):
                        self.log(f"Aborted at line {i}: {line}")
                        return
                else:
                    # keep RX buffer from filling; show anything machine says
                    self._poll_rx(ser)

                payload = line.encode("ascii", "ignore") + eol
                if not self._send(ser, payload):
                    self.log("Stopped by user")
                    return

                sent += len(payload)
                self.q.put(("progress", (i, total, sent, total_bytes)))
            while ser.out_waiting:
                time.sleep(0.1)
            self._drain(ser)
            self.log("Finished sending")


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NIMROD DNC")
        self.q = queue.Queue()
        self.watcher = None
        self.sender = None
        self.pending = None
        self.port_map = {}
        self._build()
        self._load_config()
        self.refresh_ports()
        self.refresh_files()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.poll_queue)
        try:
            self.iconbitmap("app_icon.ico")
        except:
            pass
        
        

    # ---- layout
    def _build(self):
        pad = {"padx": 6, "pady": 3}
        self.v = {k: tk.StringVar() for k in (
            "dir", "file", "port", "baud", "databits", "parity", "stopbits",
            "flow", "handshake", "eol", "line_delay_ms", "ack_timeout", "chunk")}
        self.v_strip = tk.BooleanVar(value=False)
        self.v_fanuc = tk.BooleanVar(value=True)
        self.v_confirm = tk.BooleanVar(value=True)
        self.v_preset = tk.StringVar()

        # Watch folder
        f = ttk.LabelFrame(self, text="Watch")
        f.grid(row=0, column=0, sticky="ew", **pad)
        f.columnconfigure(1, weight=1)
        ttk.Label(f, text="Folder").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(f, textvariable=self.v["dir"]).grid(row=0, column=1, sticky="ew", **pad)
        ttk.Button(f, text="Browse…", command=self.pick_dir).grid(row=0, column=2, **pad)
        ttk.Label(f, text="File").grid(row=1, column=0, sticky="w", **pad)
        ttk.Entry(f, textvariable=self.v["file"], state="readonly").grid(row=1, column=1, sticky="ew", **pad)
        fb = ttk.Frame(f)
        fb.grid(row=1, column=2)
        ttk.Button(fb, text="Select…", command=self.pick_file).pack(side="left", **pad)
        ttk.Button(fb, text="Any file", command=self.clear_file).pack(side="left", **pad)
        """
        self.cb_file = ttk.Combobox(f, textvariable=self.v["file"], state="readonly")
        self.cb_file.grid(row=1, column=1, sticky="ew", **pad)
        ttk.Button(f, text="Refresh", command=self.refresh_files).grid(row=1, column=2, **pad)
        """
        ttk.Checkbutton(f, text="Ask for confirmation before auto-starting a job",
                        variable=self.v_confirm).grid(row=2, column=0, columnspan=3, sticky="w", **pad)

        
        s = ttk.LabelFrame(self, text="Serial / UART")
        s.grid(row=1, column=0, sticky="ew", **pad)
        s.columnconfigure(1, weight=1)

        ttk.Label(s, text="Preset").grid(row=0, column=0, sticky="w", **pad)
        self.cb_preset = ttk.Combobox(s, textvariable=self.v_preset)  # editable: type a name, then Save
        self.cb_preset.grid(row=0, column=1, sticky="ew", **pad)
        self.cb_preset.bind("<<ComboboxSelected>>", self.preset_load)
        pb = ttk.Frame(s)
        pb.grid(row=0, column=2)
        ttk.Button(pb, text="Save", command=self.preset_save).pack(side="left", **pad)
        ttk.Button(pb, text="Delete", command=self.preset_delete).pack(side="left", **pad)
        
        ttk.Label(s, text="Port").grid(row=1, column=0, sticky="w", **pad)
        self.cb_port = ttk.Combobox(s, textvariable=self.v["port"], state="readonly", width=44)
        self.cb_port.grid(row=1, column=1, sticky="ew", **pad)
        ttk.Button(s, text="Rescan", command=self.refresh_ports).grid(row=1, column=2, **pad)

        grid = ttk.Frame(s)
        grid.grid(row=2, column=0, columnspan=3, sticky="ew")
        def combo(label, key, values, r, c, editable=False):
            ttk.Label(grid, text=label).grid(row=r, column=c * 2, sticky="w", **pad)
            ttk.Combobox(grid, textvariable=self.v[key], values=values, width=18,
                         state="normal" if editable else "readonly"
                         ).grid(row=r, column=c * 2 + 1, sticky="w", **pad)
        combo("Baud", "baud", BAUDS, 0, 0, editable=True)
        combo("Data bits", "databits", ["5", "6", "7", "8"], 0, 1)
        combo("Parity", "parity", list(PARITY), 1, 0)
        combo("Stop bits", "stopbits", list(STOPBITS), 1, 1)
        combo("Flow control", "flow", FLOW, 2, 0)
        combo("Line ending", "eol", list(EOLS), 2, 1)
        combo("Handshake", "handshake", HANDSHAKE, 3, 0)
        ttk.Label(grid, text="Line delay (ms)").grid(row=3, column=2, sticky="w", **pad)
        ttk.Entry(grid, textvariable=self.v["line_delay_ms"], width=8).grid(row=3, column=3, sticky="w", **pad)
        ttk.Label(grid, text="'ok' timeout (s)").grid(row=4, column=0, sticky="w", **pad)
        ttk.Entry(grid, textvariable=self.v["ack_timeout"], width=8).grid(row=4, column=1, sticky="w", **pad)
        ttk.Checkbutton(grid, text="Strip (comments)", variable=self.v_strip
                        ).grid(row=4, column=2, columnspan=2, sticky="w", **pad)
        ttk.Label(grid, text="TX chunk (bytes)").grid(row=5, column=0, sticky="w", **pad)
        ttk.Entry(grid, textvariable=self.v["chunk"], width=8).grid(row=5, column=1, sticky="w", **pad)
        ttk.Checkbutton(grid, text="Ensure % at start/end (Fanuc)", variable=self.v_fanuc
                        ).grid(row=5, column=2, columnspan=2, sticky="w", **pad)
        #ttk.Button(grid, text="Fanuc 0i preset", command=self.apply_preset
        #           ).grid(row=6, column=0, columnspan=2, sticky="w", **pad)

        # Controls
        c = ttk.Frame(self)
        c.grid(row=2, column=0, sticky="ew", **pad)
        self.btn_enable = ttk.Button(c, text="Enable auto-feed", command=self.toggle_watch)
        self.btn_enable.pack(side="left", **pad)
        self.btn_send = ttk.Button(c, text="Send file now…", command=self.send_manual)
        self.btn_send.pack(side="left", **pad)
        self.btn_pause = ttk.Button(c, text="Pause", command=self.toggle_pause, state="disabled")
        self.btn_pause.pack(side="left", **pad)
        self.btn_stop = ttk.Button(c, text="Stop", command=self.stop_job, state="disabled")
        self.btn_stop.pack(side="left", **pad)
        self.lbl_status = ttk.Label(c, text="Idle")
        self.lbl_status.pack(side="right", **pad)

        self.progress = ttk.Progressbar(self, mode="determinate")
        self.progress.grid(row=3, column=0, sticky="ew", **pad)

        self.log_box = tk.Text(self, height=14, state="disabled", wrap="none")
        self.log_box.grid(row=4, column=0, sticky="nsew", **pad)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)

        self.cb_port.bind("<<ComboboxSelected>>", lambda e: None)

    # ---- helpers
    def log(self, msg):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", time.strftime("%H:%M:%S ") + msg + "\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def cfg(self):
        c = {k: v.get() for k, v in self.v.items()}
        c["port"] = self.port_map.get(c["port"], c["port"])
        c["strip_comments"] = self.v_strip.get()
        c["fanuc_percent"] = self.v_fanuc.get()
        return c

    def pick_dir(self):
        d = filedialog.askdirectory(initialdir=self.v["dir"].get() or str(Path.home()))
        if d:
            self.v["dir"].set(d)
            self.refresh_files()

    def refresh_files(self):
        """Fall back to 'any file' if the selected file isn't in the folder."""
        sel = self.v["file"].get()
        if sel != ANY_FILE and not (Path(self.v["dir"].get() or ".") / sel).is_file():
            self.v["file"].set(ANY_FILE)

    def pick_file(self):
        exts = " ".join(f"*{e}" for e in sorted(GCODE_EXTS))
        p = filedialog.askopenfilename(
            initialdir=self.v["dir"].get() or str(Path.home()),
            filetypes=[("G-code files", exts), ("All files", "*.*")])
        if p:
            self.v["dir"].set(str(Path(p).parent))  # keep folder and file consistent
            self.v["file"].set(Path(p).name)
            self._restart_watch()

    def clear_file(self):
        self.v["file"].set(ANY_FILE)
        self._restart_watch()

    def _restart_watch(self):
        """If auto-feed is on, restart it so it watches the new selection."""
        if self.watcher:
            self.toggle_watch()
            self.toggle_watch()
    
    def refresh_ports(self):
        self.port_map = {f"{dev} - {desc}": dev for dev, desc in list_serial_ports()}
        self.cb_port["values"] = list(self.port_map)
        cur = self.v["port"].get()
        if cur not in self.port_map:
            match = next((k for k, d in self.port_map.items() if d == cur), None)
            self.v["port"].set(match or (next(iter(self.port_map), "")))

    # ---- config persistence
    def _load_config(self):
        defaults = {"dir": "", "file": ANY_FILE, "port": "", "baud": "9600",
                    "databits": "7", "parity": "Even", "stopbits": "2",
                    "flow": APP_XON, "handshake": HANDSHAKE[0], "eol": "LF (\\n)",
                    "line_delay_ms": "0", "ack_timeout": "10", "chunk": "4"}
        try:
            defaults.update(json.loads(CONFIG_PATH.read_text()))
        except (OSError, ValueError):
            pass
        for k, v in self.v.items():
            v.set(defaults.get(k, ""))
        self.v_strip.set(defaults.get("strip_comments", False))
        self.v_fanuc.set(defaults.get("fanuc_percent", True))
        self.v_confirm.set(defaults.get("confirm", True))
        saved = defaults.get("presets")
        self.presets = saved if isinstance(saved, dict) else json.loads(json.dumps(DEFAULT_PRESETS))
        self.cb_preset["values"] = sorted(self.presets)
        self.v_preset.set(defaults.get("preset", ""))

    def _save_config(self):
        data = {k: v.get() for k, v in self.v.items()}
        data["strip_comments"] = self.v_strip.get()
        data["fanuc_percent"] = self.v_fanuc.get()
        data["confirm"] = self.v_confirm.get()
        data["presets"] = self.presets
        data["preset"] = self.v_preset.get()
        
        try:
            CONFIG_PATH.write_text(json.dumps(data, indent=2))
        except OSError:
            pass

    # ---- serial presets (stored in the json)
    def preset_values(self):
        d = {k: self.v[k].get() for k in PRESET_KEYS}
        d["port"] = self.port_map.get(d["port"], d["port"])  # store "COM5", not the display text
        d["strip_comments"] = self.v_strip.get()
        d["fanuc_percent"] = self.v_fanuc.get()
        return d

    def preset_load(self, event=None):
        name = self.v_preset.get()
        p = self.presets.get(name)
        if not p:
            return
        self.refresh_ports()  # pick up adapters plugged in since startup
        for k in PRESET_KEYS:
            if k != "port" and k in p:
                self.v[k].set(p[k])
        self.v_strip.set(p.get("strip_comments", False))
        self.v_fanuc.set(p.get("fanuc_percent", True))
        dev = p.get("port", "")
        if dev:
            match = next((disp for disp, d in self.port_map.items() if d == dev), None)
            if match:
                self.v["port"].set(match)
            else:
                self.log(f"Preset port {dev} not found; port selection left unchanged")
        self.log(f"Loaded preset '{name}'")
        self._save_config()

    def preset_save(self):
        name = self.v_preset.get().strip()
        if not name:
            messagebox.showerror("Preset", "Type a preset name in the box first.")
            return
        if name in self.presets and not messagebox.askyesno("Preset", f"Overwrite preset '{name}'?"):
            return
        self.presets[name] = self.preset_values()
        self.v_preset.set(name)
        self.cb_preset["values"] = sorted(self.presets)
        self._save_config()
        self.log(f"Saved preset '{name}'")

    def preset_delete(self):
        name = self.v_preset.get().strip()
        if name not in self.presets:
            return
        if not messagebox.askyesno("Preset", f"Delete preset '{name}'?"):
            return
        del self.presets[name]
        self.v_preset.set("")
        self.cb_preset["values"] = sorted(self.presets)
        self._save_config()
        self.log(f"Deleted preset '{name}'")

    # ---- watching
    def toggle_watch(self):
        if self.watcher:
            self.watcher.stop_evt.set()
            self.watcher = None
            self.btn_enable.configure(text="Enable auto-feed")
            self.lbl_status.configure(text="Idle")
            self.log("Auto-feed disabled")
            return
        d = self.v["dir"].get()
        if not Path(d).is_dir():
            messagebox.showerror("Folder", "Choose a valid folder to monitor first.")
            return
        sel = self.v["file"].get()
        self.watcher = Watcher(d, None if sel == ANY_FILE else sel, self.q)
        self.watcher.start()
        self.btn_enable.configure(text="Disable auto-feed")
        self.lbl_status.configure(text="Watching…")
        self.log(f"Watching {d} ({'any file' if sel == ANY_FILE else sel})")

    def send_manual(self):
        sel = self.v["file"].get()
        if sel != ANY_FILE:  # a specific file is selected: send it, no dialog
            p = Path(self.v["dir"].get()) / sel
            if not p.is_file():
                messagebox.showerror("File", f"{p} was not found.")
                return
            self.start_job(str(p))
            return
        p = filedialog.askopenfilename(initialdir=self.v["dir"].get() or None)
        if p:
            self.start_job(p)

    # ---- job control
    def start_job(self, path):
        if self.sender and self.sender.is_alive():
            self.pending = path
            self.log(f"Job running; queued {Path(path).name}")
            return
        cfg = self.cfg()
        if not cfg["port"]:
            messagebox.showerror("Serial", "No serial port selected.")
            return
        self._save_config()
        self.progress["value"] = 0
        self.sender = Sender(path, cfg, self.q)
        self.sender.start()
        self.btn_pause.configure(state="normal", text="Pause")
        self.btn_stop.configure(state="normal")
        self.lbl_status.configure(text=f"Sending {Path(path).name}")

    def toggle_pause(self):
        if not self.sender:
            return
        paused = self.btn_pause.cget("text") == "Pause"
        self.sender.pause(paused)
        self.btn_pause.configure(text="Resume" if paused else "Pause")
        self.log("Paused" if paused else "Resumed")

    def stop_job(self):
        self.pending = None
        if self.sender:
            self.sender.stop()

    def on_file_event(self, path):
        self.log(f"Detected change: {Path(path).name}")
        if self.sender and self.sender.is_alive():
            self.pending = path
            self.log("Job already running; will offer new file when it finishes")
            return
        if self.v_confirm.get() and not messagebox.askyesno(
                "Start job?", f"Send {Path(path).name} to the machine now?"):
            return
        self.start_job(path)

    # ---- UI event pump
    def poll_queue(self):
        try:
            while True:
                kind, data = self.q.get_nowait()
                if kind == "log":
                    self.log(data)
                elif kind == "file":
                    self.on_file_event(data)
                elif kind == "progress":
                    i, total, sent, total_bytes = data
                    pct = 100.0 * sent / total_bytes if total_bytes else 100.0
                    self.progress["maximum"] = max(total_bytes, 1)
                    self.progress["value"] = sent
                    self.lbl_status.configure(
                        text=f"Line {i:,}/{total:,}  |  {fmt_bytes_pair(sent, total_bytes)} ({pct:.1f}%)")
                elif kind == "done":
                    self.btn_pause.configure(state="disabled", text="Pause")
                    self.btn_stop.configure(state="disabled")
                    self.lbl_status.configure(text="Watching…" if self.watcher else "Idle")
                    if self.pending:
                        p, self.pending = self.pending, None
                        self.on_file_event(p)
        except queue.Empty:
            pass
        self.after(100, self.poll_queue)

    def on_close(self):
        self._save_config()
        if self.watcher:
            self.watcher.stop_evt.set()
        if self.sender:
            self.sender.stop()
        self.destroy()


if __name__ == "__main__":
    ctypes.windll.shcore.SetProcessDpiAwareness(True)
    App().mainloop()
