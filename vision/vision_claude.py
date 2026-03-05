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
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import queue

# ─────────────────────────────── Costanti ────────────────────────────────── #

HEADER         = 0xAA
PKT_TRACKING   = 0x01
PKT_CALIB      = 0x02
BAUD_RATE      = 115200

LOWER_GREEN = np.array([50, 35, 127])
UPPER_GREEN = np.array([77, 255, 255])

LOWER_BALL = np.array([5, 80, 120])
UPPER_BALL = np.array([35, 255, 255])

VIDEO_W, VIDEO_H = 640, 480

# ─────────────────────────────── Packet util ─────────────────────────────── #

def build_packet(pkt_type: int, arg1: int, arg2: int) -> bytes:
    """Costruisce l'11 byte secondo il protocollo definito."""
    body = struct.pack('<BBii', HEADER, pkt_type, int(arg1), int(arg2))
    checksum = 0
    for b in body:
        checksum ^= b
    return body + struct.pack('B', checksum & 0xFF)

def packet_hex(pkt: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in pkt)

# ─────────────────────────────── Vision thread ───────────────────────────── #

class VisionThread(threading.Thread):
    """Cattura frames, esegue tracking e invia coordinate."""

    def __init__(self, cam_idx: int, frame_q: queue.Queue,
                 info_q: queue.Queue, serial_ref, running_ev: threading.Event):
        super().__init__(daemon=True)
        self.cam_idx    = cam_idx
        self.frame_q    = frame_q   # (annotated_frame, mask_frame)
        self.info_q     = info_q    # dict con dati da mostrare in GUI
        self.serial_ref = serial_ref
        self.running_ev = running_ev
        self.active     = False     # True = invia pacchetti

    def run(self):
        cap = cv2.VideoCapture(self.cam_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  VIDEO_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_H)

        while self.running_ev.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # Find Platform
            mask_w = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
            conts_plat, _ = cv2.findContours(mask_w, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            plat_center = None
            plat_radius = 0

            if conts_plat:
                c = max(conts_plat, key=cv2.contourArea)
                ((px, py), pr) = cv2.minEnclosingCircle(c)
                if pr > 50:
                    plat_center = (int(px), int(py))
                    plat_radius = int(pr)
                    cv2.circle(frame, plat_center, plat_radius, (255, 80, 0), 2)
                    cv2.drawMarker(frame, plat_center, (255, 80, 0), cv2.MARKER_CROSS, 20, 2)

            # Find Ball inside the platform 
            mask_b = cv2.inRange(hsv, LOWER_BALL, UPPER_BALL)
            
            # Apply Region Of Interest to find the ball
            if plat_center:
                roi_mask = np.zeros(mask_b.shape, dtype=np.uint8)
                cv2.circle(roi_mask, plat_center, max(0, plat_radius - 2), 255, -1)
                mask_b = cv2.bitwise_and(mask_b, roi_mask)
            else:
                mask_b = np.zeros_like(mask_b)

            mask_b = cv2.erode(mask_b,  None, iterations=1)
            mask_b = cv2.dilate(mask_b, None, iterations=4)
            
            conts_ball, _ = cv2.findContours(mask_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            rel_x = rel_y = 0
            ball_found = False

            if conts_ball and plat_center:
                c = max(conts_ball, key=cv2.contourArea)
                M = cv2.moments(c)
                # m00 > 10 evita di rilevare singoli pixel di rumore
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

                    # Send packet only if tracking active
                    if self.active:
                        pkt = build_packet(PKT_TRACKING, rel_x, rel_y)
                        try:
                            if self.serial_ref[0]:
                                self.serial_ref[0].write(pkt)
                        except Exception: 
                            pass
                        
                        # Aggiornamento coda
                        self.info_q.put({
                            'type': 'tracking',
                            'rel_x': rel_x, 'rel_y': rel_y,
                            'pkt': packet_hex(pkt) if 'packet_hex' in globals() else "",
                            'ball': ball_found,
                            'plat': plat_center is not None
                        })

            # Overlay status
            status_col = (0, 220, 80) if (ball_found and plat_center) else (0, 60, 220)
            cv2.putText(frame, "TRACKING ON" if self.active else "PREVIEW",
                        (VIDEO_W - 160, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, status_col, 2)

            # Metti frame nella coda (non bloccante)
            if not self.frame_q.full():
                # Inviamo sia il frame che la maschera filtrata per debug visivo nella GUI
                self.frame_q.put((frame.copy(), mask_b.copy()))

        cap.release()

# ─────────────────────────────── Applicazione GUI ────────────────────────── #

class App(tk.Tk):

    DARK_BG   = "#0d1117"
    PANEL_BG  = "#161b22"
    ACCENT    = "#00e5a0"
    ACCENT2   = "#0095ff"
    TEXT      = "#e6edf3"
    MUTED     = "#8b949e"
    RED       = "#f85149"
    BORDER    = "#30363d"
    FONT_MONO = ("Courier New", 10)
    FONT_UI   = ("Segoe UI", 10) if __import__('sys').platform == "win32" \
                else ("SF Pro Display", 10)

    def __init__(self):
        super().__init__()
        self.title("Ball Controller — ESP32 Interface")
        self.configure(bg=self.DARK_BG)
        self.resizable(False, False)

        # Stato seriale: lista[0] = oggetto Serial o None
        self.ser_ref   = [None]
        self.connected = False

        # Code comunicazione con VisionThread
        self.frame_q = queue.Queue(maxsize=2)
        self.info_q  = queue.Queue(maxsize=50)
        self.run_ev  = threading.Event()

        self.vision_thread = None

        # Log packets
        self.pkt_log = []

        self._build_ui()
        self._refresh_ports()
        self._poll_frames()
        self._poll_info()

    # ─────────────────────── Layout principale ──────────────────────────── #

    def _build_ui(self):
        # ── Header ──────────────────────────────────────────────────────── #
        hdr = tk.Frame(self, bg="#0a0f15", pady=10)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="⬡  BALL CONTROLLER", bg="#0a0f15", fg=self.ACCENT,
                 font=("Courier New", 16, "bold")).pack(side=tk.LEFT, padx=18)
        tk.Label(hdr, text="ESP32 Serial Protocol v1.0",
                 bg="#0a0f15", fg=self.MUTED, font=self.FONT_MONO).pack(
                 side=tk.LEFT, padx=4)

        # ── Corpo principale ─────────────────────────────────────────────── #
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

    # ─────────────────────── Pannello connessione ────────────────────────── #

    def _build_connection_panel(self, parent):
        frm = self._card(parent, "SERIAL CONNECTION")

        row = tk.Frame(frm, bg=self.PANEL_BG)
        row.pack(fill=tk.X, pady=4)

        tk.Label(row, text="Port", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_UI, width=6, anchor='w').pack(side=tk.LEFT)

        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(row, textvariable=self.port_var,
                                     width=18, state="readonly")
        self._style_combobox()
        self.port_cb.pack(side=tk.LEFT, padx=(4, 6))

        btn_ref = tk.Button(row, text="↺", bg=self.BORDER, fg=self.ACCENT,
                            relief=tk.FLAT, font=("Courier New", 12, "bold"),
                            cursor="hand2", command=self._refresh_ports)
        btn_ref.pack(side=tk.LEFT)

        row2 = tk.Frame(frm, bg=self.PANEL_BG)
        row2.pack(fill=tk.X, pady=(6, 4))

        tk.Label(row2, text="Baud", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_UI, width=6, anchor='w').pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(BAUD_RATE))
        tk.Entry(row2, textvariable=self.baud_var, width=12,
                 bg=self.BORDER, fg=self.TEXT, insertbackground=self.TEXT,
                 relief=tk.FLAT, font=self.FONT_MONO).pack(side=tk.LEFT, padx=(4, 0))

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

    # ─────────────────────── Video panel ────────────────────────────────── #

    def _build_video_panel(self, parent):
        frm = self._card(parent, "CAMERA FEED")

        cam_row = tk.Frame(frm, bg=self.PANEL_BG)
        cam_row.pack(fill=tk.X, pady=(0, 6))
        tk.Label(cam_row, text="Camera idx", bg=self.PANEL_BG,
                 fg=self.MUTED, font=self.FONT_UI).pack(side=tk.LEFT)
        self.cam_idx_var = tk.IntVar(value=1)
        tk.Spinbox(cam_row, from_=0, to=4, textvariable=self.cam_idx_var,
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

        # Canvas video principale
        self.video_canvas = tk.Canvas(frm, width=VIDEO_W, height=VIDEO_H,
                                      bg="#000000", highlightthickness=1,
                                      highlightbackground=self.BORDER)
        self.video_canvas.pack()

        # Placeholder
        self.video_canvas.create_text(VIDEO_W//2, VIDEO_H//2,
                                      text="CAMERA SPENTA",
                                      fill=self.MUTED, font=("Courier New", 14))

    # ─────────────────────── Tabs Tracking / Calibrazione ────────────────── #

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

        nb = ttk.Notebook(parent, style='Dark.TNotebook', width=340)
        nb.pack(fill=tk.X)
        nb.bind("<<NotebookTabChanged>>", self._on_tab_change)
        self.notebook = nb

        # Tab 0 — Tracking
        tab_t = tk.Frame(nb, bg=self.PANEL_BG, padx=12, pady=12)
        nb.add(tab_t, text="  TRACKING  ")
        self._build_tracking_tab(tab_t)

        # Tab 1 — Calibrazione
        tab_c = tk.Frame(nb, bg=self.PANEL_BG, padx=12, pady=12)
        nb.add(tab_c, text="  CALIBRAZIONE  ")
        self._build_calib_tab(tab_c)

    def _on_tab_change(self, event):
        if self.vision_thread and self.vision_thread.is_alive():
            idx = self.notebook.index(self.notebook.select())
            # Ferma invio automatico se si passa a calibrazione
            if idx == 1:
                self.vision_thread.active = False
                self.track_btn.config(text="▶  AVVIA TRACKING",
                                      bg=self.ACCENT, fg=self.DARK_BG)

    # ─── Tracking tab ─────────────────────────────────────────────────── #

    def _build_tracking_tab(self, parent):
        info = tk.Frame(parent, bg=self.PANEL_BG)
        info.pack(fill=tk.X, pady=(0, 10))

        desc = ("Il tracking rileva automaticamente la piattaforma\n"
                "(bordo bianco) e la pallina (arancione).\n"
                "Premere AVVIA per iniziare l'invio all'ESP32.")
        tk.Label(info, text=desc, bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 9), justify=tk.LEFT).pack(anchor='w')

        sep = tk.Frame(parent, bg=self.BORDER, height=1)
        sep.pack(fill=tk.X, pady=8)

        # Coordinate live
        coords = tk.Frame(parent, bg=self.PANEL_BG)
        coords.pack(fill=tk.X)

        for label, attr in [("REL X", "lbl_rx"), ("REL Y", "lbl_ry")]:
            row = tk.Frame(coords, bg=self.PANEL_BG)
            row.pack(fill=tk.X, pady=3)
            tk.Label(row, text=f"{label}:", bg=self.PANEL_BG,
                     fg=self.MUTED, font=self.FONT_MONO, width=7,
                     anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text="---", bg=self.PANEL_BG,
                           fg=self.ACCENT, font=("Courier New", 13, "bold"))
            lbl.pack(side=tk.LEFT)
            setattr(self, attr, lbl)

        # Stato rilevamento
        det_row = tk.Frame(parent, bg=self.PANEL_BG)
        det_row.pack(fill=tk.X, pady=(8, 4))
        self.lbl_plat = tk.Label(det_row, text="● Piattaforma: N/D",
                                 bg=self.PANEL_BG, fg=self.MUTED,
                                 font=self.FONT_MONO)
        self.lbl_plat.pack(anchor='w')
        self.lbl_ball = tk.Label(det_row, text="● Pallina:      N/D",
                                 bg=self.PANEL_BG, fg=self.MUTED,
                                 font=self.FONT_MONO)
        self.lbl_ball.pack(anchor='w')

        sep2 = tk.Frame(parent, bg=self.BORDER, height=1)
        sep2.pack(fill=tk.X, pady=8)

        # HSV tunign
        self._build_hsv_tuning(parent)

        sep3 = tk.Frame(parent, bg=self.BORDER, height=1)
        sep3.pack(fill=tk.X, pady=8)

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

        sliders = [
            ("H min", "hsv_h_min", 5,   0,   180),
            ("H max", "hsv_h_max", 35,  0,   180),
            ("S min", "hsv_s_min", 120, 0,   255),
            ("V min", "hsv_v_min", 150, 0,   255),
        ]
        for label, attr, default, lo, hi in sliders:
            row = tk.Frame(parent, bg=self.PANEL_BG)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=label, bg=self.PANEL_BG, fg=self.MUTED,
                     font=self.FONT_MONO, width=7, anchor='w').pack(side=tk.LEFT)
            var = tk.IntVar(value=default)
            s = tk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL,
                         variable=var, length=180,
                         bg=self.PANEL_BG, fg=self.TEXT,
                         troughcolor=self.BORDER,
                         highlightthickness=0, bd=0,
                         sliderrelief=tk.FLAT,
                         command=lambda v, a=attr: self._update_hsv(a, int(v)))
            s.pack(side=tk.LEFT)
            setattr(self, attr, var)

    def _update_hsv(self, attr, val):
        global LOWER_BALL, UPPER_BALL
        mapping = {
            'hsv_h_min': (LOWER_BALL, 0),
            'hsv_h_max': (UPPER_BALL, 0),
            'hsv_s_min': (LOWER_BALL, 1),
            'hsv_v_min': (LOWER_BALL, 2),
        }
        if attr in mapping:
            arr, idx = mapping[attr]
            arr[idx] = val

    # ─── Calibrazione tab ─────────────────────────────────────────────── #

    def _build_calib_tab(self, parent):
        desc = ("Seleziona il servo (0-2) e imposta l'angolo\ntarget.\n"
                "Premere INVIA per mandare il pacchetto.")
        tk.Label(parent, text=desc, bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 9), justify=tk.LEFT).pack(anchor='w')

        sep = tk.Frame(parent, bg=self.BORDER, height=1)
        sep.pack(fill=tk.X, pady=10)

        # Selezione servo
        row1 = tk.Frame(parent, bg=self.PANEL_BG)
        row1.pack(fill=tk.X, pady=4)
        tk.Label(row1, text="Servo ID", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)

        self.servo_var = tk.IntVar(value=0)
        for i in range(3):
            rb = tk.Radiobutton(row1, text=f" {i} ", variable=self.servo_var,
                                value=i, bg=self.PANEL_BG, fg=self.TEXT,
                                selectcolor=self.ACCENT2,
                                activebackground=self.PANEL_BG,
                                font=self.FONT_MONO,
                                command=self._update_calib_preview)
            rb.pack(side=tk.LEFT, padx=4)

        # Angolo slider
        row2 = tk.Frame(parent, bg=self.PANEL_BG)
        row2.pack(fill=tk.X, pady=(12, 4))
        tk.Label(row2, text="Angolo (°)", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)

        self.angle_var = tk.IntVar(value=90)
        self.angle_scale = tk.Scale(row2, from_=0, to=180,
                                    orient=tk.HORIZONTAL,
                                    variable=self.angle_var, length=200,
                                    bg=self.PANEL_BG, fg=self.TEXT,
                                    troughcolor=self.BORDER,
                                    highlightthickness=0, bd=0,
                                    sliderrelief=tk.FLAT,
                                    command=lambda v: self._update_calib_preview())
        self.angle_scale.pack(side=tk.LEFT)

        # Angolo display numerico
        row3 = tk.Frame(parent, bg=self.PANEL_BG)
        row3.pack(fill=tk.X, pady=4)
        tk.Label(row3, text="Angolo attuale:", bg=self.PANEL_BG,
                 fg=self.MUTED, font=self.FONT_MONO).pack(side=tk.LEFT)
        self.lbl_angle_val = tk.Label(row3, text="90°", bg=self.PANEL_BG,
                                      fg=self.ACCENT2,
                                      font=("Courier New", 13, "bold"))
        self.lbl_angle_val.pack(side=tk.LEFT, padx=8)

        # Entry manuale angolo
        row4 = tk.Frame(parent, bg=self.PANEL_BG)
        row4.pack(fill=tk.X, pady=4)
        tk.Label(row4, text="Imposta °:", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=10, anchor='w').pack(side=tk.LEFT)
        self.angle_entry = tk.Entry(row4, width=6, bg=self.BORDER,
                                    fg=self.TEXT, insertbackground=self.TEXT,
                                    relief=tk.FLAT, font=self.FONT_MONO)
        self.angle_entry.insert(0, "90")
        self.angle_entry.pack(side=tk.LEFT, padx=4)
        tk.Button(row4, text="SET", bg=self.BORDER, fg=self.ACCENT,
                  relief=tk.FLAT, font=self.FONT_MONO,
                  cursor="hand2",
                  command=self._set_angle_from_entry).pack(side=tk.LEFT)

        sep2 = tk.Frame(parent, bg=self.BORDER, height=1)
        sep2.pack(fill=tk.X, pady=10)

        # Servo diagram
        self._build_servo_diagram(parent)

        sep3 = tk.Frame(parent, bg=self.BORDER, height=1)
        sep3.pack(fill=tk.X, pady=8)

        # Preset rapidi
        tk.Label(parent, text="PRESET RAPIDI", bg=self.PANEL_BG,
                 fg=self.MUTED, font=("Courier New", 9, "bold")).pack(anchor='w')
        preset_row = tk.Frame(parent, bg=self.PANEL_BG)
        preset_row.pack(fill=tk.X, pady=4)
        for angle in [0, 45, 90, 135, 180]:
            tk.Button(preset_row, text=f"{angle}°",
                      bg=self.BORDER, fg=self.TEXT,
                      relief=tk.FLAT, font=self.FONT_MONO,
                      cursor="hand2",
                      command=lambda a=angle: self._apply_preset(a)
                      ).pack(side=tk.LEFT, padx=2)

        sep4 = tk.Frame(parent, bg=self.BORDER, height=1)
        sep4.pack(fill=tk.X, pady=8)

        self.calib_btn = tk.Button(parent, text="📡  INVIA CALIBRAZIONE",
                                   bg=self.ACCENT2, fg="#ffffff",
                                   relief=tk.FLAT,
                                   font=("Courier New", 10, "bold"),
                                   cursor="hand2", padx=12, pady=7,
                                   command=self._send_calib)
        self.calib_btn.pack(fill=tk.X)

    def _build_servo_diagram(self, parent):
        """Mini canvas che mostra visivamente l'angolo del servo selezionato."""
        self.servo_canvas = tk.Canvas(parent, width=310, height=90,
                                      bg="#0a0f15", highlightthickness=0)
        self.servo_canvas.pack(pady=4)
        self._draw_servo_diagram()

    def _draw_servo_diagram(self):
        c   = self.servo_canvas
        c.delete("all")
        angle = self.angle_var.get()
        servo = self.servo_var.get()
        w, h  = 310, 90

        c.create_text(10, 10, text=f"SERVO {servo}", anchor='nw',
                      fill=self.ACCENT2, font=("Courier New", 9, "bold"))
        c.create_text(w-10, 10, text=f"{angle}°", anchor='ne',
                      fill=self.ACCENT, font=("Courier New", 11, "bold"))

        # Arco 180°
        cx, cy, r = w//2, h-15, 55
        c.create_arc(cx-r, cy-r, cx+r, cy+r,
                     start=0, extent=180, style=tk.ARC,
                     outline=self.BORDER, width=2)

        # Marcatori 0 / 90 / 180
        import math
        for a, label in [(0, "0°"), (90, "90°"), (180, "180°")]:
            rad = math.radians(180 - a)
            x1 = cx + (r+2)*math.cos(rad)
            y1 = cy - (r+2)*math.sin(rad)
            x2 = cx + (r+10)*math.cos(rad)
            y2 = cy - (r+10)*math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill=self.MUTED, width=1)
            lx = cx + (r+20)*math.cos(rad)
            ly = cy - (r+20)*math.sin(rad)
            c.create_text(lx, ly, text=label, fill=self.MUTED,
                          font=("Courier New", 7))

        # Freccia servo
        rad = math.radians(180 - angle)
        ex = cx + r*math.cos(rad)
        ey = cy - r*math.sin(rad)
        c.create_line(cx, cy, ex, ey,
                      fill=self.ACCENT2, width=3, arrow=tk.LAST)
        c.create_oval(cx-5, cy-5, cx+5, cy+5,
                      fill=self.ACCENT2, outline="")

    def _update_calib_preview(self):
        angle = self.angle_var.get()
        self.lbl_angle_val.config(text=f"{angle}°")
        self._draw_servo_diagram()

    def _set_angle_from_entry(self):
        try:
            a = int(self.angle_entry.get())
            a = max(0, min(180, a))
            self.angle_var.set(a)
            self._update_calib_preview()
        except ValueError:
            pass

    def _apply_preset(self, angle):
        self.angle_var.set(angle)
        self.angle_entry.delete(0, tk.END)
        self.angle_entry.insert(0, str(angle))
        self._update_calib_preview()

    # ─────────────────────── Packet log ──────────────────────────────────── #

    def _build_packet_log(self, parent):
        frm = self._card(parent, "PACKET LOG")

        self.log_text = tk.Text(frm, width=42, height=8,
                                bg="#0a0f15", fg=self.ACCENT,
                                font=("Courier New", 9),
                                relief=tk.FLAT, state=tk.DISABLED,
                                insertbackground=self.TEXT)
        self.log_text.pack(fill=tk.BOTH)

        self.log_text.tag_configure("tracking", foreground=self.ACCENT)
        self.log_text.tag_configure("calib",    foreground=self.ACCENT2)
        self.log_text.tag_configure("system",   foreground=self.MUTED)
        self.log_text.tag_configure("error",    foreground=self.RED)

        clear_btn = tk.Button(frm, text="CLEAR LOG", bg=self.BORDER,
                              fg=self.MUTED, relief=tk.FLAT,
                              font=("Courier New", 8), cursor="hand2",
                              command=self._clear_log)
        clear_btn.pack(anchor='e', pady=(4, 0))

    def _log(self, msg: str, tag: str = "system"):
        ts = time.strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{ts}] {msg}\n", tag)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _clear_log(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.config(state=tk.DISABLED)

    # ─────────────────────── Status bar ──────────────────────────────────── #

    def _build_status_bar(self):
        bar = tk.Frame(self, bg="#0a0f15", pady=5)
        bar.pack(fill=tk.X, side=tk.BOTTOM)

        self.status_var = tk.StringVar(value="Pronto.")
        tk.Label(bar, textvariable=self.status_var,
                 bg="#0a0f15", fg=self.MUTED,
                 font=("Courier New", 9), anchor='w').pack(
                 side=tk.LEFT, padx=12)

        self.pkt_count_var = tk.StringVar(value="PKT: 0")
        tk.Label(bar, textvariable=self.pkt_count_var,
                 bg="#0a0f15", fg=self.MUTED,
                 font=("Courier New", 9)).pack(side=tk.RIGHT, padx=12)

        self.pkt_count = 0

    # ─────────────────────── Helper card ──────────────────────────────────── #

    def _card(self, parent, title: str) -> tk.Frame:
        outer = tk.Frame(parent, bg=self.DARK_BG, pady=4)
        outer.pack(fill=tk.X, pady=4)
        tk.Label(outer, text=title, bg=self.DARK_BG, fg=self.MUTED,
                 font=("Courier New", 8, "bold")).pack(anchor='w', padx=2)
        inner = tk.Frame(outer, bg=self.PANEL_BG, padx=12, pady=10,
                         relief=tk.FLAT, bd=1)
        inner.pack(fill=tk.X)
        return inner

    def _style_combobox(self):
        style = ttk.Style(self)
        style.theme_use('default')
        style.configure('TCombobox',
                        fieldbackground=self.BORDER,
                        background=self.BORDER,
                        foreground=self.TEXT,
                        selectbackground=self.ACCENT2,
                        selectforeground="#ffffff",
                        arrowcolor=self.MUTED)

    # ─────────────────────── Azioni seriale ──────────────────────────────── #

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
                self.conn_status.config(text=f"● {port} @ {baud}", fg=self.ACCENT)
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

    # ─────────────────────── Azioni camera ───────────────────────────────── #

    def _toggle_camera(self):
        if self.vision_thread and self.vision_thread.is_alive():
            self.run_ev.clear()
            self.cam_btn.config(text="▶  AVVIA CAMERA", bg=self.ACCENT,
                                fg=self.DARK_BG)
            self._log("Camera arrestata.", "system")
            # Reset canvas
            self.video_canvas.delete("all")
            self.video_canvas.create_text(VIDEO_W//2, VIDEO_H//2,
                                          text="CAMERA SPENTA",
                                          fill=self.MUTED,
                                          font=("Courier New", 14))
            if self.vision_thread.active:
                self._stop_tracking_ui()
        else:
            self.run_ev.set()
            self.vision_thread = VisionThread(
                self.cam_idx_var.get(),
                self.frame_q, self.info_q,
                self.ser_ref, self.run_ev
            )
            self.vision_thread.start()
            self.cam_btn.config(text="⬛  FERMA CAMERA", bg=self.RED,
                                fg="#ffffff")
            self._log("Camera avviata.", "system")

    # ─────────────────────── Tracking on/off ─────────────────────────────── #

    def _toggle_tracking(self):
        if not (self.vision_thread and self.vision_thread.is_alive()):
            messagebox.showwarning("Attenzione", "Avvia prima la camera.")
            return
        if not self.vision_thread.active:
            if not self.connected:
                if not messagebox.askyesno("Attenzione",
                    "Seriale non connessa.\nProcedere in modalità TEST (solo visualizzazione)?"):
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

    # ─────────────────────── Invio calibrazione ──────────────────────────── #

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

    # ─────────────────────── Poll frame queue ────────────────────────────── #

    def _poll_frames(self):
        try:
            while not self.frame_q.empty():
                frame, _ = self.frame_q.get_nowait()
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil = Image.fromarray(img)
                tk_img = ImageTk.PhotoImage(pil)
                self.video_canvas.delete("all")
                self.video_canvas.create_image(0, 0, anchor=tk.NW, image=tk_img)
                self.video_canvas._img = tk_img  # mantieni riferimento
        except queue.Empty:
            pass
        self.after(33, self._poll_frames)  # ~30 fps

    # ─────────────────────── Poll info queue ─────────────────────────────── #

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

    # ─────────────────────── Chiusura ────────────────────────────────────── #

    def on_close(self):
        self.run_ev.clear()
        if self.ser_ref[0]:
            self.ser_ref[0].close()
        self.destroy()


# ─────────────────────────────── Entry point ─────────────────────────────── #

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()