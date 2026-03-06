"""
Ball-on-Platform Controller
GUI per tracking visivo e calibrazione servo tramite ESP32.

Struttura pacchetto (11 byte):
  [0]     HEADER   1B  = 0xAA
  [1]     TYPE     1B  = 0x00 tracking | 0x01 calibrazione
  [2..5]  ARG1     4B  signed int32 LE  (rel_x | servo_id)
  [6..9]  ARG2     4B  signed int32 LE  (rel_y | angolo)
  [10]    CHECKSUM 1B  XOR di [0..9]
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import struct
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import queue

# ══════════════════════════════════════════════════════════════════════════════
#  COSTANTI — modifica qui per cambiare i default all'avvio
# ══════════════════════════════════════════════════════════════════════════════

HEADER       = 0xAA
PKT_TRACKING = 0x01
PKT_CALIB    = 0x02
BAUD_RATE    = 115200

# ── HSV piattaforma (verde) e pallina (arancione) ────────────────────────── #
LOWER_GREEN = np.array([50,  55, 127])
UPPER_GREEN = np.array([77, 255, 255])
LOWER_BALL  = np.array([ 5,  80, 120])
UPPER_BALL  = np.array([35, 255, 255])

VIDEO_W, VIDEO_H = 640, 480

# ── Device V4L2 ──────────────────────────────────────────────────────────── #
# Percorso del device video da controllare con v4l2-ctl
CAM_DEVICE = "/dev/video0"

# ── Default esposizione ──────────────────────────────────────────────────── #
# auto_exposure: 1 = Manuale, 3 = Auto  (semantica v4l2 UVC)
DEFAULT_AUTO_EXPOSURE    = 1          # 1 = Manuale all'avvio
DEFAULT_EXPOSURE_TIME    = 150        # exposure_time_absolute (1–2000 tipico)
EXPOSURE_TIME_MIN        = 1
EXPOSURE_TIME_MAX        = 2000

# ── Default bilanciamento del bianco ────────────────────────────────────── #
# white_balance_automatic: 0 = Manuale, 1 = Auto
DEFAULT_AUTO_WB          = 0          # 0 = Manuale all'avvio
DEFAULT_WB_TEMPERATURE   = 4600       # Kelvin (tipico 2800–6500)
WB_TEMPERATURE_MIN       = 2800
WB_TEMPERATURE_MAX       = 6500

# ══════════════════════════════════════════════════════════════════════════════
#  UTILITY — pacchetti seriali
# ══════════════════════════════════════════════════════════════════════════════

def build_packet(pkt_type: int, arg1: int, arg2: int) -> bytes:
    body = struct.pack('<BBii', HEADER, pkt_type, int(arg1), int(arg2))
    checksum = 0
    for b in body:
        checksum ^= b
    return body + struct.pack('B', checksum & 0xFF)

def packet_hex(pkt: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in pkt)

# ══════════════════════════════════════════════════════════════════════════════
#  UTILITY — controllo V4L2
# ══════════════════════════════════════════════════════════════════════════════

def v4l2_set(device: str, control: str, value: int):
    """
    Esegue: v4l2-ctl -d <device> -c <control>=<value>
    Restituisce (ok: bool, messaggio: str).
    """
    cmd = ["v4l2-ctl", "-d", device, "-c", f"{control}={value}"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
        if r.returncode == 0:
            return True, f"{control}={value}"
        else:
            return False, (r.stderr.strip() or r.stdout.strip())
    except FileNotFoundError:
        return False, "v4l2-ctl non trovato — installa v4l-utils"
    except subprocess.TimeoutExpired:
        return False, "v4l2-ctl timeout"
    except Exception as e:
        return False, str(e)

def v4l2_get(device: str, control: str):
    """
    Legge il valore corrente di un controllo V4L2.
    Restituisce (ok: bool, valore: int|None, riga: str).
    """
    cmd = ["v4l2-ctl", "-d", device, "--get-ctrl", control]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
        if r.returncode == 0:
            line = r.stdout.strip()
            val  = int(line.split(":")[-1].strip())
            return True, val, line
        else:
            return False, None, r.stderr.strip()
    except Exception as e:
        return False, None, str(e)

# ══════════════════════════════════════════════════════════════════════════════
#  VISION THREAD
# ══════════════════════════════════════════════════════════════════════════════

class VisionThread(threading.Thread):

    def __init__(self, cam_idx, frame_q, info_q, serial_ref, running_ev):
        super().__init__(daemon=True)
        self.cam_idx    = cam_idx
        self.frame_q    = frame_q
        self.info_q     = info_q
        self.serial_ref = serial_ref
        self.running_ev = running_ev
        self.active     = False

    def run(self):
        cap = cv2.VideoCapture(self.cam_idx)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  VIDEO_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_H)
        cap.set(cv2.CAP_PROP_FPS, 30)

        while self.running_ev.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # Piattaforma
            mask_w = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
            conts_plat, _ = cv2.findContours(mask_w, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
            plat_center = None
            plat_radius = 0
            if conts_plat:
                c = max(conts_plat, key=cv2.contourArea)
                ((px, py), pr) = cv2.minEnclosingCircle(c)
                if pr > 50:
                    plat_center = (int(px), int(py))
                    plat_radius = int(pr)
                    cv2.circle(frame, plat_center, plat_radius, (255, 80, 0), 2)
                    cv2.drawMarker(frame, plat_center, (255, 80, 0),
                                   cv2.MARKER_CROSS, 20, 2)

            # Pallina — solo dentro ROI piattaforma
            mask_b = cv2.inRange(hsv, LOWER_BALL, UPPER_BALL)
            if plat_center:
                roi = np.zeros(mask_b.shape, dtype=np.uint8)
                cv2.circle(roi, plat_center, max(0, plat_radius - 2), 255, -1)
                mask_b = cv2.bitwise_and(mask_b, roi)
            else:
                mask_b = np.zeros_like(mask_b)

            mask_b = cv2.erode(mask_b,  None, iterations=1)
            mask_b = cv2.dilate(mask_b, None, iterations=2)

            conts_ball, _ = cv2.findContours(mask_b, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
            rel_x = rel_y = 0
            ball_found = False

            if conts_ball and plat_center:
                c = max(conts_ball, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 10:
                    bx = int(M["m10"] / M["m00"])
                    by = int(M["m01"] / M["m00"])
                    rel_x = bx - plat_center[0]
                    rel_y = plat_center[1] - by
                    ball_found = True
                    cv2.circle(frame, (bx, by), 12, (0, 220, 80), -1)
                    cv2.line(frame, plat_center, (bx, by), (0, 220, 80), 1)
                    cv2.putText(frame, f"X:{rel_x:+4d}  Y:{rel_y:+4d}",
                                (12, 36), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 220, 80), 2)
                    if self.active:
                        pkt = build_packet(PKT_TRACKING, rel_x, rel_y)
                        try:
                            if self.serial_ref[0]:
                                self.serial_ref[0].write(pkt)
                        except Exception:
                            pass
                        self.info_q.put({
                            'type': 'tracking',
                            'rel_x': rel_x, 'rel_y': rel_y,
                            'pkt':  packet_hex(pkt),
                            'ball': ball_found,
                            'plat': plat_center is not None
                        })

            status_col = (0, 220, 80) if (ball_found and plat_center) \
                         else (0, 60, 220)
            cv2.putText(frame, "TRACKING ON" if self.active else "PREVIEW",
                        (VIDEO_W - 160, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, status_col, 2)

            if not self.frame_q.full():
                self.frame_q.put((frame.copy(), mask_b.copy()))

        cap.release()

# ══════════════════════════════════════════════════════════════════════════════
#  APPLICAZIONE GUI
# ══════════════════════════════════════════════════════════════════════════════

class App(tk.Tk):

    DARK_BG   = "#0d1117"
    PANEL_BG  = "#161b22"
    ACCENT    = "#00e5a0"
    ACCENT2   = "#0095ff"
    TEXT      = "#e6edf3"
    MUTED     = "#8b949e"
    RED       = "#f85149"
    BORDER    = "#30363d"
    YELLOW    = "#e3b341"
    FONT_MONO = ("Courier New", 10)

    def __init__(self):
        super().__init__()
        self.title("Ball Controller — ESP32 Interface")
        self.configure(bg=self.DARK_BG)
        self.resizable(False, False)

        self.ser_ref   = [None]
        self.connected = False
        self.frame_q   = queue.Queue(maxsize=2)
        self.info_q    = queue.Queue(maxsize=50)
        self.run_ev    = threading.Event()
        self.vision_thread = None
        self.pkt_log   = []
        self.pkt_count = 0

        # Stato locale controlli camera
        self._auto_exp_on = (DEFAULT_AUTO_EXPOSURE == 3)
        self._auto_wb_on  = (DEFAULT_AUTO_WB == 1)

        self._build_ui()
        self._refresh_ports()
        self._poll_frames()
        self._poll_info()
        # Applica default camera 500 ms dopo il boot della GUI
        self.after(500, self._apply_camera_defaults)

    # ══════════════════════════════════════════════════════════════════════ #
    #  LAYOUT
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_ui(self):
        hdr = tk.Frame(self, bg="#0a0f15", pady=10)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="⬡  BALL CONTROLLER", bg="#0a0f15", fg=self.ACCENT,
                 font=("Courier New", 16, "bold")).pack(side=tk.LEFT, padx=18)
        tk.Label(hdr, text="ESP32 Serial Protocol v1.0",
                 bg="#0a0f15", fg=self.MUTED,
                 font=self.FONT_MONO).pack(side=tk.LEFT, padx=4)

        body = tk.Frame(self, bg=self.DARK_BG)
        body.pack(fill=tk.BOTH, padx=14, pady=(6, 14))

        left  = tk.Frame(body, bg=self.DARK_BG)
        right = tk.Frame(body, bg=self.DARK_BG)
        left.pack(side=tk.LEFT, fill=tk.BOTH)
        right.pack(side=tk.LEFT, fill=tk.BOTH, padx=(12, 0))

        self._build_connection_panel(left)
        self._build_video_panel(left)
        self._build_tabs(right)
        self._build_packet_log(right)
        self._build_status_bar()

    # ── Connessione seriale ───────────────────────────────────────────────── #

    def _build_connection_panel(self, parent):
        frm = self._card(parent, "SERIAL CONNECTION")

        r = tk.Frame(frm, bg=self.PANEL_BG)
        r.pack(fill=tk.X, pady=4)
        tk.Label(r, text="Port", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=6, anchor='w').pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(r, textvariable=self.port_var,
                                     width=18, state="readonly")
        self._style_combobox()
        self.port_cb.pack(side=tk.LEFT, padx=(4, 6))
        tk.Button(r, text="↺", bg=self.BORDER, fg=self.ACCENT,
                  relief=tk.FLAT, font=("Courier New", 12, "bold"),
                  cursor="hand2",
                  command=self._refresh_ports).pack(side=tk.LEFT)

        r2 = tk.Frame(frm, bg=self.PANEL_BG)
        r2.pack(fill=tk.X, pady=(6, 4))
        tk.Label(r2, text="Baud", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=6, anchor='w').pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(BAUD_RATE))
        tk.Entry(r2, textvariable=self.baud_var, width=12,
                 bg=self.BORDER, fg=self.TEXT, insertbackground=self.TEXT,
                 relief=tk.FLAT, font=self.FONT_MONO).pack(side=tk.LEFT,
                                                            padx=(4, 0))

        self.conn_btn = tk.Button(frm, text="CONNECT", bg=self.ACCENT2,
                                  fg="#ffffff", relief=tk.FLAT,
                                  font=("Courier New", 10, "bold"),
                                  cursor="hand2", padx=12, pady=5,
                                  command=self._toggle_connection)
        self.conn_btn.pack(fill=tk.X, pady=(8, 2))
        self.conn_status = tk.Label(frm, text="● Disconnesso",
                                    bg=self.PANEL_BG, fg=self.RED,
                                    font=self.FONT_MONO)
        self.conn_status.pack(anchor='w', pady=(2, 0))

    # ── Video ─────────────────────────────────────────────────────────────── #

    def _build_video_panel(self, parent):
        frm = self._card(parent, "CAMERA FEED")

        cr = tk.Frame(frm, bg=self.PANEL_BG)
        cr.pack(fill=tk.X, pady=(0, 6))
        tk.Label(cr, text="Camera idx", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO).pack(side=tk.LEFT)
        self.cam_idx_var = tk.IntVar(value=1)
        tk.Spinbox(cr, from_=0, to=4, textvariable=self.cam_idx_var,
                   width=4, bg=self.BORDER, fg=self.TEXT,
                   buttonbackground=self.BORDER, relief=tk.FLAT,
                   font=self.FONT_MONO).pack(side=tk.LEFT, padx=6)

        self.cam_btn = tk.Button(frm, text="▶  AVVIA CAMERA",
                                 bg=self.ACCENT, fg=self.DARK_BG,
                                 relief=tk.FLAT,
                                 font=("Courier New", 10, "bold"),
                                 cursor="hand2", padx=12, pady=5,
                                 command=self._toggle_camera)
        self.cam_btn.pack(fill=tk.X, pady=(0, 6))

        self.video_canvas = tk.Canvas(frm, width=VIDEO_W, height=VIDEO_H,
                                      bg="#000000", highlightthickness=1,
                                      highlightbackground=self.BORDER)
        self.video_canvas.pack()
        self.video_canvas.create_text(VIDEO_W // 2, VIDEO_H // 2,
                                      text="CAMERA SPENTA",
                                      fill=self.MUTED,
                                      font=("Courier New", 14))

    # ── Tabs ──────────────────────────────────────────────────────────────── #

    def _build_tabs(self, parent):
        style = ttk.Style(self)
        style.theme_use('default')
        style.configure('Dark.TNotebook',
                        background=self.DARK_BG, borderwidth=0)
        style.configure('Dark.TNotebook.Tab',
                        background=self.PANEL_BG, foreground=self.MUTED,
                        font=("Courier New", 10, "bold"),
                        padding=[14, 6], borderwidth=0)
        style.map('Dark.TNotebook.Tab',
                  background=[('selected', self.DARK_BG)],
                  foreground=[('selected', self.ACCENT)])

        nb = ttk.Notebook(parent, style='Dark.TNotebook', width=370)
        nb.pack(fill=tk.X)
        nb.bind("<<NotebookTabChanged>>", self._on_tab_change)
        self.notebook = nb

        tab_t = tk.Frame(nb, bg=self.PANEL_BG, padx=12, pady=12)
        nb.add(tab_t, text="  TRACKING  ")
        self._build_tracking_tab(tab_t)

        tab_c = tk.Frame(nb, bg=self.PANEL_BG, padx=12, pady=12)
        nb.add(tab_c, text="  CALIBRAZIONE  ")
        self._build_calib_tab(tab_c)

        tab_cam = tk.Frame(nb, bg=self.PANEL_BG, padx=12, pady=12)
        nb.add(tab_cam, text="  CAMERA  ")
        self._build_camera_tab(tab_cam)

    def _on_tab_change(self, event):
        if self.vision_thread and self.vision_thread.is_alive():
            if self.notebook.index(self.notebook.select()) == 1:
                self.vision_thread.active = False
                self.track_btn.config(text="▶  AVVIA TRACKING",
                                      bg=self.ACCENT, fg=self.DARK_BG)

    # ══════════════════════════════════════════════════════════════════════ #
    #  TAB TRACKING
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_tracking_tab(self, parent):
        desc = ("Il tracking rileva la piattaforma (bordo verde)\n"
                "e la pallina (arancione).\n"
                "Premere AVVIA per iniziare l'invio all'ESP32.")
        tk.Label(parent, text=desc, bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 9), justify=tk.LEFT).pack(anchor='w')
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        for label, attr in [("REL X", "lbl_rx"), ("REL Y", "lbl_ry")]:
            row = tk.Frame(parent, bg=self.PANEL_BG)
            row.pack(fill=tk.X, pady=3)
            tk.Label(row, text=f"{label}:", bg=self.PANEL_BG, fg=self.MUTED,
                     font=self.FONT_MONO, width=7,
                     anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text="---", bg=self.PANEL_BG, fg=self.ACCENT,
                           font=("Courier New", 13, "bold"))
            lbl.pack(side=tk.LEFT)
            setattr(self, attr, lbl)

        det = tk.Frame(parent, bg=self.PANEL_BG)
        det.pack(fill=tk.X, pady=(8, 4))
        self.lbl_plat = tk.Label(det, text="● Piattaforma: N/D",
                                  bg=self.PANEL_BG, fg=self.MUTED,
                                  font=self.FONT_MONO)
        self.lbl_plat.pack(anchor='w')
        self.lbl_ball = tk.Label(det, text="● Pallina:      N/D",
                                  bg=self.PANEL_BG, fg=self.MUTED,
                                  font=self.FONT_MONO)
        self.lbl_ball.pack(anchor='w')

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)
        self._build_hsv_tuning(parent)
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        self.track_btn = tk.Button(parent, text="▶  AVVIA TRACKING",
                                   bg=self.ACCENT, fg=self.DARK_BG,
                                   relief=tk.FLAT,
                                   font=("Courier New", 10, "bold"),
                                   cursor="hand2", padx=12, pady=7,
                                   command=self._toggle_tracking)
        self.track_btn.pack(fill=tk.X)

    def _build_hsv_tuning(self, parent):
        tk.Label(parent, text="HSV BALL TUNE", bg=self.PANEL_BG,
                 fg=self.MUTED, font=("Courier New", 9, "bold")).pack(anchor='w')
        for label, attr, default, lo, hi in [
            ("H min", "hsv_h_min",  5,   0, 180),
            ("H max", "hsv_h_max", 35,   0, 180),
            ("S min", "hsv_s_min", 80,   0, 255),
            ("V min", "hsv_v_min", 120,  0, 255),
        ]:
            row = tk.Frame(parent, bg=self.PANEL_BG)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=label, bg=self.PANEL_BG, fg=self.MUTED,
                     font=self.FONT_MONO, width=7,
                     anchor='w').pack(side=tk.LEFT)
            var = tk.IntVar(value=default)
            tk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL,
                     variable=var, length=190,
                     bg=self.PANEL_BG, fg=self.TEXT,
                     troughcolor=self.BORDER, highlightthickness=0, bd=0,
                     sliderrelief=tk.FLAT,
                     command=lambda v, a=attr: self._update_hsv(a, int(v))
                     ).pack(side=tk.LEFT)
            setattr(self, attr, var)

    def _update_hsv(self, attr, val):
        global LOWER_BALL, UPPER_BALL
        m = {'hsv_h_min': (LOWER_BALL, 0), 'hsv_h_max': (UPPER_BALL, 0),
             'hsv_s_min': (LOWER_BALL, 1), 'hsv_v_min': (LOWER_BALL, 2)}
        if attr in m:
            m[attr][0][m[attr][1]] = val

    # ══════════════════════════════════════════════════════════════════════ #
    #  TAB CALIBRAZIONE
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_calib_tab(self, parent):
        tk.Label(parent,
                 text="Seleziona il servo (0-2) e imposta l'angolo target.\n"
                      "Premere INVIA per mandare il pacchetto.",
                 bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 9), justify=tk.LEFT).pack(anchor='w')
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=10)

        r1 = tk.Frame(parent, bg=self.PANEL_BG)
        r1.pack(fill=tk.X, pady=4)
        tk.Label(r1, text="Servo ID", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.servo_var = tk.IntVar(value=0)
        for i in range(3):
            tk.Radiobutton(r1, text=f" {i} ", variable=self.servo_var, value=i,
                           bg=self.PANEL_BG, fg=self.TEXT,
                           selectcolor=self.ACCENT2,
                           activebackground=self.PANEL_BG,
                           font=self.FONT_MONO,
                           command=self._update_calib_preview
                           ).pack(side=tk.LEFT, padx=4)

        r2 = tk.Frame(parent, bg=self.PANEL_BG)
        r2.pack(fill=tk.X, pady=(12, 4))
        tk.Label(r2, text="Angolo (°)", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.angle_var = tk.IntVar(value=90)
        tk.Scale(r2, from_=0, to=180, orient=tk.HORIZONTAL,
                 variable=self.angle_var, length=210,
                 bg=self.PANEL_BG, fg=self.TEXT,
                 troughcolor=self.BORDER, highlightthickness=0, bd=0,
                 sliderrelief=tk.FLAT,
                 command=lambda v: self._update_calib_preview()
                 ).pack(side=tk.LEFT)

        r3 = tk.Frame(parent, bg=self.PANEL_BG)
        r3.pack(fill=tk.X, pady=4)
        tk.Label(r3, text="Angolo attuale:", bg=self.PANEL_BG,
                 fg=self.MUTED, font=self.FONT_MONO).pack(side=tk.LEFT)
        self.lbl_angle_val = tk.Label(r3, text="90°", bg=self.PANEL_BG,
                                      fg=self.ACCENT2,
                                      font=("Courier New", 13, "bold"))
        self.lbl_angle_val.pack(side=tk.LEFT, padx=8)

        r4 = tk.Frame(parent, bg=self.PANEL_BG)
        r4.pack(fill=tk.X, pady=4)
        tk.Label(r4, text="Imposta °:", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.angle_entry = tk.Entry(r4, width=6, bg=self.BORDER,
                                    fg=self.TEXT, insertbackground=self.TEXT,
                                    relief=tk.FLAT, font=self.FONT_MONO)
        self.angle_entry.insert(0, "90")
        self.angle_entry.pack(side=tk.LEFT, padx=4)
        tk.Button(r4, text="SET", bg=self.BORDER, fg=self.ACCENT,
                  relief=tk.FLAT, font=self.FONT_MONO, cursor="hand2",
                  command=self._set_angle_from_entry).pack(side=tk.LEFT)

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=10)
        self._build_servo_diagram(parent)
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        tk.Label(parent, text="PRESET RAPIDI", bg=self.PANEL_BG,
                 fg=self.MUTED, font=("Courier New", 9, "bold")).pack(anchor='w')
        pr = tk.Frame(parent, bg=self.PANEL_BG)
        pr.pack(fill=tk.X, pady=4)
        for a in [0, 45, 90, 135, 180]:
            tk.Button(pr, text=f"{a}°", bg=self.BORDER, fg=self.TEXT,
                      relief=tk.FLAT, font=self.FONT_MONO, cursor="hand2",
                      command=lambda v=a: self._apply_preset(v)
                      ).pack(side=tk.LEFT, padx=2)

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)
        self.calib_btn = tk.Button(parent, text="📡  INVIA CALIBRAZIONE",
                                   bg=self.ACCENT2, fg="#ffffff",
                                   relief=tk.FLAT,
                                   font=("Courier New", 10, "bold"),
                                   cursor="hand2", padx=12, pady=7,
                                   command=self._send_calib)
        self.calib_btn.pack(fill=tk.X)

    def _build_servo_diagram(self, parent):
        self.servo_canvas = tk.Canvas(parent, width=330, height=90,
                                      bg="#0a0f15", highlightthickness=0)
        self.servo_canvas.pack(pady=4)
        self._draw_servo_diagram()

    def _draw_servo_diagram(self):
        import math
        c = self.servo_canvas
        c.delete("all")
        angle = self.angle_var.get()
        servo = self.servo_var.get()
        w, h = 330, 90
        c.create_text(10, 10, text=f"SERVO {servo}", anchor='nw',
                      fill=self.ACCENT2, font=("Courier New", 9, "bold"))
        c.create_text(w - 10, 10, text=f"{angle}°", anchor='ne',
                      fill=self.ACCENT, font=("Courier New", 11, "bold"))
        cx, cy, r = w // 2, h - 15, 55
        c.create_arc(cx-r, cy-r, cx+r, cy+r,
                     start=0, extent=180, style=tk.ARC,
                     outline=self.BORDER, width=2)
        for a, label in [(0, "0°"), (90, "90°"), (180, "180°")]:
            rad = math.radians(180 - a)
            x1 = cx + (r + 2)  * math.cos(rad)
            y1 = cy - (r + 2)  * math.sin(rad)
            x2 = cx + (r + 10) * math.cos(rad)
            y2 = cy - (r + 10) * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill=self.MUTED, width=1)
            lx = cx + (r + 20) * math.cos(rad)
            ly = cy - (r + 20) * math.sin(rad)
            c.create_text(lx, ly, text=label, fill=self.MUTED,
                          font=("Courier New", 7))
        rad = math.radians(180 - angle)
        ex = cx + r * math.cos(rad)
        ey = cy - r * math.sin(rad)
        c.create_line(cx, cy, ex, ey, fill=self.ACCENT2, width=3,
                      arrow=tk.LAST)
        c.create_oval(cx-5, cy-5, cx+5, cy+5,
                      fill=self.ACCENT2, outline="")

    def _update_calib_preview(self):
        self.lbl_angle_val.config(text=f"{self.angle_var.get()}°")
        self._draw_servo_diagram()

    def _set_angle_from_entry(self):
        try:
            a = max(0, min(180, int(self.angle_entry.get())))
            self.angle_var.set(a)
            self._update_calib_preview()
        except ValueError:
            pass

    def _apply_preset(self, angle):
        self.angle_var.set(angle)
        self.angle_entry.delete(0, tk.END)
        self.angle_entry.insert(0, str(angle))
        self._update_calib_preview()

    # ══════════════════════════════════════════════════════════════════════ #
    #  TAB CAMERA — Esposizione e Bilanciamento del Bianco via v4l2-ctl
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_camera_tab(self, parent):

        # ── Device ─────────────────────────────────────────────────────── #
        dev_row = tk.Frame(parent, bg=self.PANEL_BG)
        dev_row.pack(fill=tk.X, pady=(0, 4))
        tk.Label(dev_row, text="Device", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=8, anchor='w').pack(side=tk.LEFT)
        self.cam_device_var = tk.StringVar(value=CAM_DEVICE)
        tk.Entry(dev_row, textvariable=self.cam_device_var, width=14,
                 bg=self.BORDER, fg=self.TEXT, insertbackground=self.TEXT,
                 relief=tk.FLAT, font=self.FONT_MONO).pack(side=tk.LEFT,
                                                            padx=4)
        tk.Button(dev_row, text="↺ LEGGI CAMERA", bg=self.BORDER,
                  fg=self.ACCENT, relief=tk.FLAT,
                  font=("Courier New", 9, "bold"), cursor="hand2",
                  command=self._read_cam_values).pack(side=tk.LEFT, padx=4)

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        # ════════════════════════════════════════════════════════════════ #
        #  ESPOSIZIONE
        # ════════════════════════════════════════════════════════════════ #
        self._section_label(parent, "ESPOSIZIONE")

        exp_tog = tk.Frame(parent, bg=self.PANEL_BG)
        exp_tog.pack(fill=tk.X, pady=(4, 6))
        tk.Label(exp_tog, text="Modalità", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.btn_auto_exp = tk.Button(
            exp_tog,
            text=self._exp_lbl(self._auto_exp_on),
            bg=self.YELLOW if self._auto_exp_on else self.ACCENT2,
            fg=self.DARK_BG, relief=tk.FLAT,
            font=("Courier New", 9, "bold"), cursor="hand2",
            padx=10, pady=3,
            command=self._toggle_auto_exposure)
        self.btn_auto_exp.pack(side=tk.LEFT, padx=4)
        v4l2_val_exp = 3 if self._auto_exp_on else 1
        self.lbl_exp_hint = tk.Label(
            exp_tog, text=f"(v4l2: auto_exposure={v4l2_val_exp})",
            bg=self.PANEL_BG, fg=self.MUTED, font=("Courier New", 8))
        self.lbl_exp_hint.pack(side=tk.LEFT, padx=6)

        exp_sl_row = tk.Frame(parent, bg=self.PANEL_BG)
        exp_sl_row.pack(fill=tk.X, pady=2)
        tk.Label(exp_sl_row, text="Exp. Time", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.exp_var = tk.IntVar(value=DEFAULT_EXPOSURE_TIME)
        self.exp_slider = tk.Scale(
            exp_sl_row,
            from_=EXPOSURE_TIME_MIN, to=EXPOSURE_TIME_MAX,
            orient=tk.HORIZONTAL, variable=self.exp_var, length=200,
            bg=self.PANEL_BG, fg=self.TEXT, troughcolor=self.BORDER,
            highlightthickness=0, bd=0, sliderrelief=tk.FLAT,
            state=tk.DISABLED if self._auto_exp_on else tk.NORMAL,
            command=lambda v: self._sync_exp_entry())
        self.exp_slider.pack(side=tk.LEFT)

        exp_num = tk.Frame(parent, bg=self.PANEL_BG)
        exp_num.pack(fill=tk.X, pady=(0, 2))
        tk.Label(exp_num, text="", bg=self.PANEL_BG,
                 width=10).pack(side=tk.LEFT)
        self.lbl_exp_val = tk.Label(exp_num, text=str(DEFAULT_EXPOSURE_TIME),
                                    bg=self.PANEL_BG, fg=self.ACCENT,
                                    font=("Courier New", 11, "bold"))
        self.lbl_exp_val.pack(side=tk.LEFT)
        tk.Label(exp_num, text=" μs  (exposure_time_absolute)",
                 bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 8)).pack(side=tk.LEFT)

        exp_man = tk.Frame(parent, bg=self.PANEL_BG)
        exp_man.pack(fill=tk.X, pady=(2, 2))
        tk.Label(exp_man, text="Imposta:", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.exp_entry = tk.Entry(exp_man, width=7, bg=self.BORDER,
                                  fg=self.TEXT, insertbackground=self.TEXT,
                                  relief=tk.FLAT, font=self.FONT_MONO)
        self.exp_entry.insert(0, str(DEFAULT_EXPOSURE_TIME))
        self.exp_entry.pack(side=tk.LEFT, padx=4)
        tk.Button(exp_man, text="SET", bg=self.BORDER, fg=self.ACCENT,
                  relief=tk.FLAT, font=self.FONT_MONO, cursor="hand2",
                  command=self._set_exp_from_entry).pack(side=tk.LEFT)
        tk.Button(exp_man, text="APPLICA ▶", bg=self.ACCENT2,
                  fg="#ffffff", relief=tk.FLAT,
                  font=("Courier New", 9, "bold"), cursor="hand2",
                  command=self._apply_exposure).pack(side=tk.LEFT, padx=(8, 0))

        tk.Label(parent, text="Preset rapidi:", bg=self.PANEL_BG,
                 fg=self.MUTED, font=("Courier New", 8)).pack(anchor='w',
                                                               pady=(4, 1))
        ep_r = tk.Frame(parent, bg=self.PANEL_BG)
        ep_r.pack(fill=tk.X, pady=(0, 4))
        for ev in [50, 100, 150, 300, 600, 1000]:
            tk.Button(ep_r, text=str(ev), bg=self.BORDER, fg=self.TEXT,
                      relief=tk.FLAT, font=("Courier New", 8), cursor="hand2",
                      command=lambda v=ev: self._exp_preset(v)
                      ).pack(side=tk.LEFT, padx=2)

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        # ════════════════════════════════════════════════════════════════ #
        #  BILANCIAMENTO DEL BIANCO
        # ════════════════════════════════════════════════════════════════ #
        self._section_label(parent, "BILANCIAMENTO DEL BIANCO")

        wb_tog = tk.Frame(parent, bg=self.PANEL_BG)
        wb_tog.pack(fill=tk.X, pady=(4, 6))
        tk.Label(wb_tog, text="Modalità", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.btn_auto_wb = tk.Button(
            wb_tog,
            text=self._wb_lbl(self._auto_wb_on),
            bg=self.YELLOW if self._auto_wb_on else self.ACCENT2,
            fg=self.DARK_BG, relief=tk.FLAT,
            font=("Courier New", 9, "bold"), cursor="hand2",
            padx=10, pady=3,
            command=self._toggle_auto_wb)
        self.btn_auto_wb.pack(side=tk.LEFT, padx=4)
        v4l2_val_wb = 1 if self._auto_wb_on else 0
        self.lbl_wb_hint = tk.Label(
            wb_tog,
            text=f"(v4l2: white_balance_automatic={v4l2_val_wb})",
            bg=self.PANEL_BG, fg=self.MUTED, font=("Courier New", 8))
        self.lbl_wb_hint.pack(side=tk.LEFT, padx=6)

        wb_sl_row = tk.Frame(parent, bg=self.PANEL_BG)
        wb_sl_row.pack(fill=tk.X, pady=2)
        tk.Label(wb_sl_row, text="Temp. (K)", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.wb_var = tk.IntVar(value=DEFAULT_WB_TEMPERATURE)
        self.wb_slider = tk.Scale(
            wb_sl_row,
            from_=WB_TEMPERATURE_MIN, to=WB_TEMPERATURE_MAX,
            orient=tk.HORIZONTAL, variable=self.wb_var, length=200,
            resolution=100,
            bg=self.PANEL_BG, fg=self.TEXT, troughcolor=self.BORDER,
            highlightthickness=0, bd=0, sliderrelief=tk.FLAT,
            state=tk.DISABLED if self._auto_wb_on else tk.NORMAL,
            command=lambda v: self._sync_wb_entry())
        self.wb_slider.pack(side=tk.LEFT)

        wb_num = tk.Frame(parent, bg=self.PANEL_BG)
        wb_num.pack(fill=tk.X, pady=(0, 2))
        tk.Label(wb_num, text="", bg=self.PANEL_BG,
                 width=10).pack(side=tk.LEFT)
        self.lbl_wb_val = tk.Label(wb_num,
                                   text=str(DEFAULT_WB_TEMPERATURE),
                                   bg=self.PANEL_BG, fg=self.ACCENT,
                                   font=("Courier New", 11, "bold"))
        self.lbl_wb_val.pack(side=tk.LEFT)
        tk.Label(wb_num, text=" K  ", bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 8)).pack(side=tk.LEFT)
        self.lbl_wb_desc = tk.Label(
            wb_num, text=self._k_desc(DEFAULT_WB_TEMPERATURE),
            bg=self.PANEL_BG, fg=self.MUTED, font=("Courier New", 8))
        self.lbl_wb_desc.pack(side=tk.LEFT)

        wb_man = tk.Frame(parent, bg=self.PANEL_BG)
        wb_man.pack(fill=tk.X, pady=(2, 2))
        tk.Label(wb_man, text="Imposta K:", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.wb_entry = tk.Entry(wb_man, width=7, bg=self.BORDER,
                                 fg=self.TEXT, insertbackground=self.TEXT,
                                 relief=tk.FLAT, font=self.FONT_MONO)
        self.wb_entry.insert(0, str(DEFAULT_WB_TEMPERATURE))
        self.wb_entry.pack(side=tk.LEFT, padx=4)
        tk.Button(wb_man, text="SET", bg=self.BORDER, fg=self.ACCENT,
                  relief=tk.FLAT, font=self.FONT_MONO, cursor="hand2",
                  command=self._set_wb_from_entry).pack(side=tk.LEFT)
        tk.Button(wb_man, text="APPLICA ▶", bg=self.ACCENT2,
                  fg="#ffffff", relief=tk.FLAT,
                  font=("Courier New", 9, "bold"), cursor="hand2",
                  command=self._apply_wb).pack(side=tk.LEFT, padx=(8, 0))

        tk.Label(parent, text="Preset rapidi:", bg=self.PANEL_BG,
                 fg=self.MUTED, font=("Courier New", 8)).pack(anchor='w',
                                                               pady=(4, 1))
        wb_pr = tk.Frame(parent, bg=self.PANEL_BG)
        wb_pr.pack(fill=tk.X, pady=(0, 4))
        for k, lbl in [(2800, "Tungst."), (3200, "Incand."),
                       (4000, "Fluor."), (5500, "Giorno"), (6500, "Nuv.")]:
            tk.Button(wb_pr, text=lbl, bg=self.BORDER, fg=self.TEXT,
                      relief=tk.FLAT, font=("Courier New", 8), cursor="hand2",
                      command=lambda v=k: self._wb_preset(v)
                      ).pack(side=tk.LEFT, padx=2)

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        # ── Feedback v4l2 ──────────────────────────────────────────────── #
        self._section_label(parent, "FEEDBACK V4L2")
        self.lbl_v4l2 = tk.Label(parent, text="—",
                                  bg=self.PANEL_BG, fg=self.MUTED,
                                  font=("Courier New", 8),
                                  wraplength=340, justify=tk.LEFT)
        self.lbl_v4l2.pack(anchor='w', pady=(2, 0))

    # ══════════════════════════════════════════════════════════════════════ #
    #  LOGICA CAMERA — v4l2
    # ══════════════════════════════════════════════════════════════════════ #

    @staticmethod
    def _exp_lbl(auto): return "● AUTO" if auto else "● MANUALE"

    @staticmethod
    def _wb_lbl(auto):  return "● AUTO" if auto else "● MANUALE"

    @staticmethod
    def _k_desc(k):
        if k < 3200:  return "Tungsteno / luce calda"
        if k < 4500:  return "Fluorescente"
        if k < 5500:  return "Luce del giorno"
        if k < 6200:  return "Flash / parzialmente nuvoloso"
        return "Cielo limpido / luce fredda"

    def _v4l2_run(self, fn):
        threading.Thread(target=fn, daemon=True).start()

    def _v4l2_fb(self, msg, ok=True):
        color = self.ACCENT if ok else self.RED
        self.after(0, lambda: self.lbl_v4l2.config(text=msg, fg=color))
        self._log(msg, "system" if ok else "error")

    def _apply_camera_defaults(self):
        dev = self.cam_device_var.get()
        def _do():
            v4l2_set(dev, "auto_exposure", DEFAULT_AUTO_EXPOSURE)
            if DEFAULT_AUTO_EXPOSURE == 1:
                v4l2_set(dev, "exposure_time_absolute", DEFAULT_EXPOSURE_TIME)
            v4l2_set(dev, "white_balance_automatic", DEFAULT_AUTO_WB)
            if DEFAULT_AUTO_WB == 0:
                v4l2_set(dev, "white_balance_temperature",
                         DEFAULT_WB_TEMPERATURE)
            self._v4l2_fb("✔ Default camera applicati.", ok=True)
        self._v4l2_run(_do)

    # ── Toggle Auto Exposure ─────────────────────────────────────────────── #

    def _toggle_auto_exposure(self):
        self._auto_exp_on = not self._auto_exp_on
        v4l2_val = 3 if self._auto_exp_on else 1
        self.btn_auto_exp.config(
            text=self._exp_lbl(self._auto_exp_on),
            bg=self.YELLOW if self._auto_exp_on else self.ACCENT2)
        self.lbl_exp_hint.config(text=f"(v4l2: auto_exposure={v4l2_val})")
        self.exp_slider.config(
            state=tk.DISABLED if self._auto_exp_on else tk.NORMAL)
        dev = self.cam_device_var.get()
        def _do():
            ok, msg = v4l2_set(dev, "auto_exposure", v4l2_val)
            self._v4l2_fb(
                f"{'✔' if ok else '✘'} auto_exposure={v4l2_val} → {msg}", ok)
        self._v4l2_run(_do)

    # ── Toggle Auto White Balance ────────────────────────────────────────── #

    def _toggle_auto_wb(self):
        self._auto_wb_on = not self._auto_wb_on
        v4l2_val = 1 if self._auto_wb_on else 0
        self.btn_auto_wb.config(
            text=self._wb_lbl(self._auto_wb_on),
            bg=self.YELLOW if self._auto_wb_on else self.ACCENT2)
        self.lbl_wb_hint.config(
            text=f"(v4l2: white_balance_automatic={v4l2_val})")
        self.wb_slider.config(
            state=tk.DISABLED if self._auto_wb_on else tk.NORMAL)
        dev = self.cam_device_var.get()
        def _do():
            ok, msg = v4l2_set(dev, "white_balance_automatic", v4l2_val)
            self._v4l2_fb(
                f"{'✔' if ok else '✘'} white_balance_automatic={v4l2_val} → {msg}",
                ok)
        self._v4l2_run(_do)

    # ── Esposizione ──────────────────────────────────────────────────────── #

    def _sync_exp_entry(self):
        v = self.exp_var.get()
        self.lbl_exp_val.config(text=str(v))
        self.exp_entry.delete(0, tk.END)
        self.exp_entry.insert(0, str(v))

    def _set_exp_from_entry(self):
        try:
            v = max(EXPOSURE_TIME_MIN,
                    min(EXPOSURE_TIME_MAX, int(self.exp_entry.get())))
            self.exp_var.set(v)
            self._sync_exp_entry()
        except ValueError:
            pass

    def _apply_exposure(self):
        if self._auto_exp_on:
            self._v4l2_fb(
                "⚠ Disattiva prima l'autoesposizione.", ok=False)
            return
        v = self.exp_var.get()
        dev = self.cam_device_var.get()
        def _do():
            ok, msg = v4l2_set(dev, "exposure_time_absolute", v)
            self._v4l2_fb(
                f"{'✔' if ok else '✘'} exposure_time_absolute={v} → {msg}", ok)
        self._v4l2_run(_do)

    def _exp_preset(self, val):
        self.exp_var.set(val)
        self._sync_exp_entry()
        self._apply_exposure()

    # ── White Balance ────────────────────────────────────────────────────── #

    def _sync_wb_entry(self):
        v = self.wb_var.get()
        self.lbl_wb_val.config(text=str(v))
        self.lbl_wb_desc.config(text=self._k_desc(v))
        self.wb_entry.delete(0, tk.END)
        self.wb_entry.insert(0, str(v))

    def _set_wb_from_entry(self):
        try:
            v = max(WB_TEMPERATURE_MIN,
                    min(WB_TEMPERATURE_MAX, int(self.wb_entry.get())))
            self.wb_var.set(v)
            self._sync_wb_entry()
        except ValueError:
            pass

    def _apply_wb(self):
        if self._auto_wb_on:
            self._v4l2_fb(
                "⚠ Disattiva prima il bilanciamento automatico.", ok=False)
            return
        v = self.wb_var.get()
        dev = self.cam_device_var.get()
        def _do():
            ok, msg = v4l2_set(dev, "white_balance_temperature", v)
            self._v4l2_fb(
                f"{'✔' if ok else '✘'} white_balance_temperature={v}K → {msg}",
                ok)
        self._v4l2_run(_do)

    def _wb_preset(self, val):
        self.wb_var.set(val)
        self._sync_wb_entry()
        self._apply_wb()

    # ── Leggi valori correnti ────────────────────────────────────────────── #

    def _read_cam_values(self):
        dev = self.cam_device_var.get()
        def _do():
            results = []
            for ctrl in ["auto_exposure", "exposure_time_absolute",
                         "white_balance_automatic",
                         "white_balance_temperature"]:
                ok, val, line = v4l2_get(dev, ctrl)
                results.append(f"{'✔' if ok else '✘'} {line}")
                if ok and val is not None:
                    if ctrl == "auto_exposure":
                        self._auto_exp_on = (val == 3)
                        self.after(0, lambda v=val: self._sync_exp_toggle_ui(v))
                    elif ctrl == "exposure_time_absolute":
                        clamped = max(EXPOSURE_TIME_MIN,
                                      min(EXPOSURE_TIME_MAX, val))
                        self.after(0, lambda v=clamped: (
                            self.exp_var.set(v), self._sync_exp_entry()))
                    elif ctrl == "white_balance_automatic":
                        self._auto_wb_on = (val == 1)
                        self.after(0, lambda v=val: self._sync_wb_toggle_ui(v))
                    elif ctrl == "white_balance_temperature":
                        clamped = max(WB_TEMPERATURE_MIN,
                                      min(WB_TEMPERATURE_MAX, val))
                        self.after(0, lambda v=clamped: (
                            self.wb_var.set(v), self._sync_wb_entry()))
            self._v4l2_fb("  |  ".join(results), ok=True)
        self._v4l2_run(_do)

    def _sync_exp_toggle_ui(self, v4l2_val):
        self._auto_exp_on = (v4l2_val == 3)
        self.btn_auto_exp.config(
            text=self._exp_lbl(self._auto_exp_on),
            bg=self.YELLOW if self._auto_exp_on else self.ACCENT2)
        self.lbl_exp_hint.config(text=f"(v4l2: auto_exposure={v4l2_val})")
        self.exp_slider.config(
            state=tk.DISABLED if self._auto_exp_on else tk.NORMAL)

    def _sync_wb_toggle_ui(self, v4l2_val):
        self._auto_wb_on = (v4l2_val == 1)
        self.btn_auto_wb.config(
            text=self._wb_lbl(self._auto_wb_on),
            bg=self.YELLOW if self._auto_wb_on else self.ACCENT2)
        self.lbl_wb_hint.config(
            text=f"(v4l2: white_balance_automatic={v4l2_val})")
        self.wb_slider.config(
            state=tk.DISABLED if self._auto_wb_on else tk.NORMAL)

    # ══════════════════════════════════════════════════════════════════════ #
    #  PACKET LOG
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_packet_log(self, parent):
        frm = self._card(parent, "PACKET LOG")
        self.log_text = tk.Text(frm, width=44, height=8,
                                bg="#0a0f15", fg=self.ACCENT,
                                font=("Courier New", 9),
                                relief=tk.FLAT, state=tk.DISABLED,
                                insertbackground=self.TEXT)
        self.log_text.pack(fill=tk.BOTH)
        self.log_text.tag_configure("tracking", foreground=self.ACCENT)
        self.log_text.tag_configure("calib",    foreground=self.ACCENT2)
        self.log_text.tag_configure("system",   foreground=self.MUTED)
        self.log_text.tag_configure("error",    foreground=self.RED)
        tk.Button(frm, text="CLEAR LOG", bg=self.BORDER, fg=self.MUTED,
                  relief=tk.FLAT, font=("Courier New", 8), cursor="hand2",
                  command=self._clear_log).pack(anchor='e', pady=(4, 0))

    def _log(self, msg, tag="system"):
        ts = time.strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{ts}] {msg}\n", tag)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _clear_log(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.config(state=tk.DISABLED)

    # ══════════════════════════════════════════════════════════════════════ #
    #  STATUS BAR
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_status_bar(self):
        bar = tk.Frame(self, bg="#0a0f15", pady=5)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        self.status_var = tk.StringVar(value="Pronto.")
        tk.Label(bar, textvariable=self.status_var, bg="#0a0f15",
                 fg=self.MUTED, font=("Courier New", 9),
                 anchor='w').pack(side=tk.LEFT, padx=12)
        self.pkt_count_var = tk.StringVar(value="PKT: 0")
        tk.Label(bar, textvariable=self.pkt_count_var, bg="#0a0f15",
                 fg=self.MUTED, font=("Courier New", 9)
                 ).pack(side=tk.RIGHT, padx=12)

    # ══════════════════════════════════════════════════════════════════════ #
    #  HELPERS GUI
    # ══════════════════════════════════════════════════════════════════════ #

    def _section_label(self, parent, text):
        tk.Label(parent, text=text, bg=self.PANEL_BG, fg=self.YELLOW,
                 font=("Courier New", 9, "bold")).pack(anchor='w',
                                                       pady=(2, 0))

    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=self.DARK_BG, pady=4)
        outer.pack(fill=tk.X, pady=4)
        tk.Label(outer, text=title, bg=self.DARK_BG, fg=self.MUTED,
                 font=("Courier New", 8, "bold")).pack(anchor='w', padx=2)
        inner = tk.Frame(outer, bg=self.PANEL_BG, padx=12, pady=10,
                         relief=tk.FLAT, bd=1)
        inner.pack(fill=tk.X)
        return inner

    def _style_combobox(self):
        s = ttk.Style(self)
        s.theme_use('default')
        s.configure('TCombobox',
                    fieldbackground=self.BORDER, background=self.BORDER,
                    foreground=self.TEXT, selectbackground=self.ACCENT2,
                    selectforeground="#ffffff", arrowcolor=self.MUTED)

    # ══════════════════════════════════════════════════════════════════════ #
    #  SERIALE
    # ══════════════════════════════════════════════════════════════════════ #

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports:
            self.port_var.set(ports[0])
        self._log(f"Porte trovate: {', '.join(ports) if ports else 'nessuna'}")

    def _toggle_connection(self):
        if not self.connected:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Errore", "Nessuna porta selezionata.")
                return
            try:
                baud = int(self.baud_var.get())
                s = serial.Serial(port, baud, timeout=0)
                self.ser_ref[0] = s
                self.connected  = True
                self.conn_btn.config(text="DISCONNECT", bg=self.RED)
                self.conn_status.config(text=f"● {port} @ {baud}",
                                        fg=self.ACCENT)
                self._log(f"Connesso a {port} @ {baud} baud", "system")
                self.status_var.set(f"Connesso: {port}")
            except serial.SerialException as e:
                messagebox.showerror("Errore seriale", str(e))
        else:
            if self.ser_ref[0]:
                self.ser_ref[0].close()
                self.ser_ref[0] = None
            self.connected = False
            self.conn_btn.config(text="CONNECT", bg=self.ACCENT2)
            self.conn_status.config(text="● Disconnesso", fg=self.RED)
            self._log("Disconnesso.", "system")
            self.status_var.set("Disconnesso.")

    # ══════════════════════════════════════════════════════════════════════ #
    #  CAMERA ON/OFF
    # ══════════════════════════════════════════════════════════════════════ #

    def _toggle_camera(self):
        if self.vision_thread and self.vision_thread.is_alive():
            self.run_ev.clear()
            self.cam_btn.config(text="▶  AVVIA CAMERA",
                                bg=self.ACCENT, fg=self.DARK_BG)
            self._log("Camera arrestata.", "system")
            self.video_canvas.delete("all")
            self.video_canvas.create_text(VIDEO_W // 2, VIDEO_H // 2,
                                          text="CAMERA SPENTA",
                                          fill=self.MUTED,
                                          font=("Courier New", 14))
            if self.vision_thread.active:
                self._stop_tracking_ui()
        else:
            self.run_ev.set()
            self.vision_thread = VisionThread(
                self.cam_idx_var.get(), self.frame_q, self.info_q,
                self.ser_ref, self.run_ev)
            self.vision_thread.start()
            self.cam_btn.config(text="⬛  FERMA CAMERA",
                                bg=self.RED, fg="#ffffff")
            self._log("Camera avviata.", "system")

    # ══════════════════════════════════════════════════════════════════════ #
    #  TRACKING
    # ══════════════════════════════════════════════════════════════════════ #

    def _toggle_tracking(self):
        if not (self.vision_thread and self.vision_thread.is_alive()):
            messagebox.showwarning("Attenzione", "Avvia prima la camera.")
            return
        if not self.vision_thread.active:
            if not self.connected:
                if not messagebox.askyesno(
                    "Attenzione",
                    "Seriale non connessa.\n"
                    "Procedere in modalità TEST (solo visualizzazione)?"
                ):
                    return
            self.vision_thread.active = True
            self.track_btn.config(text="⬛  STOP TRACKING",
                                  bg=self.RED, fg="#ffffff")
            self._log("Tracking AVVIATO.", "tracking")
            self.status_var.set("Tracking attivo...")
        else:
            self._stop_tracking_ui()

    def _stop_tracking_ui(self):
        if self.vision_thread:
            self.vision_thread.active = False
        self.track_btn.config(text="▶  AVVIA TRACKING",
                              bg=self.ACCENT, fg=self.DARK_BG)
        self._log("Tracking FERMATO.", "system")
        self.status_var.set("Tracking fermato.")

    # ══════════════════════════════════════════════════════════════════════ #
    #  CALIBRAZIONE
    # ══════════════════════════════════════════════════════════════════════ #

    def _send_calib(self):
        servo = self.servo_var.get()
        angle = self.angle_var.get()
        pkt   = build_packet(PKT_CALIB, servo, angle)
        msg   = f"CALIB → servo={servo} angle={angle}° | {packet_hex(pkt)}"
        if self.ser_ref[0]:
            try:
                self.ser_ref[0].write(pkt)
                self._log(msg, "calib")
                self.status_var.set(f"Calibrazione inviata: S{servo} {angle}°")
            except serial.SerialException as e:
                self._log(f"ERRORE TX: {e}", "error")
        else:
            self._log(f"[TEST] {msg}", "calib")
            self.status_var.set(f"[TEST] {msg}")
        self.pkt_count += 1
        self.pkt_count_var.set(f"PKT: {self.pkt_count}")

    # ══════════════════════════════════════════════════════════════════════ #
    #  POLL
    # ══════════════════════════════════════════════════════════════════════ #

    def _poll_frames(self):
        try:
            while not self.frame_q.empty():
                frame, _ = self.frame_q.get_nowait()
                img    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil    = Image.fromarray(img)
                tk_img = ImageTk.PhotoImage(pil)
                self.video_canvas.delete("all")
                self.video_canvas.create_image(0, 0, anchor=tk.NW,
                                               image=tk_img)
                self.video_canvas._img = tk_img
        except queue.Empty:
            pass
        self.after(33, self._poll_frames)

    def _poll_info(self):
        try:
            while not self.info_q.empty():
                info = self.info_q.get_nowait()
                if info['type'] == 'tracking':
                    self.lbl_rx.config(text=f"{info['rel_x']:+5d} px")
                    self.lbl_ry.config(text=f"{info['rel_y']:+5d} px")
                    pc = self.ACCENT if info['plat'] else self.RED
                    bc = self.ACCENT if info['ball'] else self.RED
                    self.lbl_plat.config(
                        text=f"● Piattaforma: {'OK' if info['plat'] else 'NON TROVATA'}",
                        fg=pc)
                    self.lbl_ball.config(
                        text=f"● Pallina:      {'OK' if info['ball'] else 'NON TROVATA'}",
                        fg=bc)
                    self._log(f"TX {info['pkt']}", "tracking")
                    self.pkt_count += 1
                    self.pkt_count_var.set(f"PKT: {self.pkt_count}")
        except queue.Empty:
            pass
        self.after(100, self._poll_info)

    # ══════════════════════════════════════════════════════════════════════ #
    #  CHIUSURA
    # ══════════════════════════════════════════════════════════════════════ #

    def on_close(self):
        self.run_ev.clear()
        if self.ser_ref[0]:
            self.ser_ref[0].close()
        self.destroy()


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()