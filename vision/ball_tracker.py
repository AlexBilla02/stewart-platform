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
import json
import numpy as np
from pathlib import Path
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

# ── ArUco ────────────────────────────────────────────────────────────────── #
# Dizionario usato per stampare i marker (deve corrispondere ai marker fisici)
ARUCO_DICT_ID     = cv2.aruco.DICT_4X4_50
# ID dei 4 marker incollati sulla piattaforma (modificabili nell'interfaccia)
# Con 4 marker la pallina può occluderne al massimo 1 — i 3 rimasti bastano
# per calcolare il circocentro con qualsiasi combinazione.
ARUCO_IDS_DEFAULT = [0, 1, 2, 3]
# Numero minimo di marker visibili per calcolare la piattaforma
ARUCO_MIN_VISIBLE = 3
# Margine extra (pixel) aggiunto al raggio della ROI per rilevare la pallina
# anche quando è al bordo o leggermente fuori dai marker
ROI_RADIUS_MARGIN = 18
# File di calibrazione ArUco (stessa cartella dello script)
ARUCO_CALIB_FILE  = Path(__file__).parent / "aruco_calib.json"


# ── File preset HSV ─────────────────────────────────────────────────────── #
# Il file viene creato nella stessa cartella dello script
PRESETS_FILE = Path(__file__).parent / "hsv_presets.json"
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
#  UTILITY GEOMETRICA
# ══════════════════════════════════════════════════════════════════════════════

def kasa_circle_fit(points):
    """
    Fit di un cerchio ai minimi quadrati (metodo di Kåsa) su N punti 2D.
    Robusto agli errori di posizionamento: minimizza la somma dei quadrati
    delle distanze di ogni punto dalla circonferenza.
    Restituisce (center: np.array [x,y], radius: float) oppure (None, 0).
    Richiede almeno 3 punti non collineari.
    """
    pts = np.array(points, dtype=float)
    if len(pts) < 3:
        return None, 0.0
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([x, y, np.ones(len(pts))])
    b = x**2 + y**2
    try:
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None, 0.0
    cx = result[0] / 2.0
    cy = result[1] / 2.0
    r  = float(np.sqrt(result[2] + cx**2 + cy**2))
    if r < 1:
        return None, 0.0
    return np.array([cx, cy]), r


def estimate_center_from_offsets(found: dict, offsets: dict):
    """
    Stima il centro della piattaforma usando gli offset di calibrazione.

    Per ogni marker visibile calcola:
        stima_centro = pos_rilevata - offset_calibrato

    dove offset_calibrato = pos_marker_calibrata - centro_calibrato.
    Restituisce la media di tutte le stime (una per ogni marker visibile).

    found:   {marker_id: [x, y]}   — posizioni rilevate runtime
    offsets: {marker_id: [dx, dy]} — offset salvati durante la calibrazione
    """
    estimates = []
    for mid, pos in found.items():
        if mid in offsets:
            off = np.array(offsets[mid])
            estimates.append(np.array(pos) - off)
    if not estimates:
        return None
    return np.mean(estimates, axis=0)
# ══════════════════════════════════════════════════════════════════════════════

class VisionThread(threading.Thread):
    """
    Cattura frames, rileva la piattaforma tramite 3 marker ArUco,
    individua la pallina con HSV nella ROI, invia coordinate all'ESP32.
    """

    def __init__(self, cam_idx, frame_q, info_q, serial_ref, running_ev,
                 calib_ref):
        super().__init__(daemon=True)
        self.cam_idx    = cam_idx
        self.frame_q    = frame_q
        self.info_q     = info_q
        self.serial_ref = serial_ref
        self.running_ev = running_ev
        self.calib_ref  = calib_ref   # [dict_or_None] — condiviso con GUI
        self.active     = False

    # ── Loop principale ───────────────────────────────────────────────────── #

    def run(self):
        # Inizializza ArUco nel thread
        aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        aruco_params = cv2.aruco.DetectorParameters()
        detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        cap = cv2.VideoCapture(self.cam_idx)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  VIDEO_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_H)

        while self.running_ev.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # frame_clean = frame originale, usato per HSV (mai toccato da draw)
            # frame       = copia su cui disegnare HUD e overlay
            frame_clean = frame.copy()
            calib = self.calib_ref[0]

            # ── Rileva marker ArUco ──────────────────────────────────────── #
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            plat_center  = None
            plat_radius  = 0
            markers_seen = []

            if ids is not None and calib is not None:
                target_ids = calib['marker_ids']
                found = {}

                for i, mid in enumerate(ids.flatten()):
                    if mid in target_ids:
                        ctr = corners[i][0].mean(axis=0)
                        found[mid] = ctr.tolist()
                        markers_seen.append(int(mid))
                        cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
                        cv2.circle(frame, (int(ctr[0]), int(ctr[1])),
                                   5, (0, 255, 255), -1)

                if len(found) >= ARUCO_MIN_VISIBLE:
                    # Stima centro con offset calibrati (robusto al
                    # posizionamento impreciso dei marker)
                    offsets_raw = calib.get('marker_offsets', {})
                    # Normalizza chiavi a int (JSON potrebbe averle come str)
                    offsets = {int(k): v for k, v in offsets_raw.items()}

                    center_arr = None
                    radius     = 0.0

                    if offsets:
                        center_arr = estimate_center_from_offsets(found, offsets)
                        radius     = float(calib.get('circle_radius_px',
                                           calib.get('circumradius_px', 0.0)))

                    # Fallback a Kåsa se offset assenti o stima fallita
                    if center_arr is None or radius < 20:
                        center_arr, radius = kasa_circle_fit(
                            list(found.values()))

                    if center_arr is not None and radius > 20:
                        cx_f, cy_f = float(center_arr[0]), float(center_arr[1])
                        plat_center = (int(cx_f), int(cy_f))
                        plat_radius = int(radius)

                        # BGR: (200, 0, 0) = blu
                        BLUE      = (200, 60,  0)
                        BLUE_THIN = (180, 40,  0)
                        cv2.circle(frame, plat_center, plat_radius, BLUE, 2)
                        cv2.line(frame,
                                 (plat_center[0] - plat_radius, plat_center[1]),
                                 (plat_center[0] + plat_radius, plat_center[1]),
                                 BLUE_THIN, 1, cv2.LINE_AA)
                        cv2.line(frame,
                                 (plat_center[0], plat_center[1] - plat_radius),
                                 (plat_center[0], plat_center[1] + plat_radius),
                                 BLUE_THIN, 1, cv2.LINE_AA)
                        cv2.drawMarker(frame, plat_center, BLUE,
                                       cv2.MARKER_CROSS, 20, 2)
                        n = len(found)
                        cv2.putText(frame, f"{n}/4 mrkr",
                                    (plat_center[0] + plat_radius + 4,
                                     plat_center[1]),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4, BLUE, 1)

                elif 0 < len(found) < ARUCO_MIN_VISIBLE:
                    cv2.putText(frame,
                                f"MARKER PARZIALI ({len(found)}/{len(target_ids)})",
                                (10, VIDEO_H - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 200, 200), 1)

            elif calib is None:
                cv2.putText(frame, "NON CALIBRATO — vai al tab ARUCO",
                            (10, VIDEO_H - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 80, 220), 2)

            # ── HSV pallina nella ROI ─────────────────────────────────────── #
            # Usa frame_clean (senza overlay) per non confondere l'HSV
            hsv    = cv2.cvtColor(cv2.GaussianBlur(frame_clean, (5, 5), 0),
                                  cv2.COLOR_BGR2HSV)
            mask_b = cv2.inRange(hsv, LOWER_BALL, UPPER_BALL)

            if plat_center:
                roi = np.zeros(mask_b.shape, dtype=np.uint8)
                cv2.circle(roi, plat_center,
                           plat_radius + ROI_RADIUS_MARGIN, 255, -1)
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
                            'type':    'tracking',
                            'rel_x':   rel_x, 'rel_y': rel_y,
                            'pkt':     packet_hex(pkt),
                            'ball':    ball_found,
                            'plat':    plat_center is not None,
                            'markers': markers_seen,
                        })

            # ── HUD ───────────────────────────────────────────────────────── #
            n_seen = len(markers_seen)
            m_col  = self._marker_color(n_seen)
            cv2.putText(frame, f"MARKERS {n_seen}/4",
                        (VIDEO_W - 130, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, m_col, 1)
            status_col = (0, 220, 80) if (ball_found and plat_center)                          else (0, 60, 220)
            cv2.putText(frame, "TRACKING ON" if self.active else "PREVIEW",
                        (VIDEO_W - 160, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_col, 2)

            self.info_q.put({
                'type':    'status',
                'markers': markers_seen,
                'plat':    plat_center is not None,
                'ball':    ball_found,
            })

            if not self.frame_q.full():
                self.frame_q.put((frame.copy(), mask_b.copy()))

        cap.release()

    @staticmethod
    def _marker_color(n: int):
        if n == 4: return (0, 220, 80)    # tutti e 4: verde
        if n == 3: return (0, 200, 200)   # 3/4: ciano — funziona ma degradato
        if n == 2: return (0, 140, 220)   # 2/4: blu — insufficiente
        return (0, 60, 220)               # <2: rosso-blu

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
        self.calib_ref = [None]   # [dict_or_None] — condiviso con VisionThread
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
        # Carica calibrazione ArUco (prima dei preset, già usa il log)
        self._load_aruco_calib()
        # Carica preset da file — dopo _build_ui, così log_text esiste già
        self._load_presets_from_file()
        self._refresh_preset_ui()
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

        tab_ar = tk.Frame(nb, bg=self.PANEL_BG, padx=12, pady=12)
        nb.add(tab_ar, text="  ARUCO  ")
        self._build_aruco_tab(tab_ar)

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
        desc = ("La piattaforma viene rilevata tramite 3 marker ArUco.\n"
                "La pallina viene rilevata con HSV nella ROI.\n"
                "Calibra prima i marker nel tab ARUCO.")
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
        # ── Pallina (arancione) ──────────────────────────────────────────── #
        tk.Label(parent, text="HSV BALL (arancione)",
                 bg=self.PANEL_BG, fg=self.ACCENT,
                 font=("Courier New", 9, "bold")).pack(anchor='w')

        for label, attr, default, lo, hi in [
            ("H min", "hsv_ball_h_min",  5,   0, 180),
            ("H max", "hsv_ball_h_max", 35,   0, 180),
            ("S min", "hsv_ball_s_min", 80,   0, 255),
            ("V min", "hsv_ball_v_min", 120,  0, 255),
        ]:
            row = tk.Frame(parent, bg=self.PANEL_BG)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=label, bg=self.PANEL_BG, fg=self.MUTED,
                     font=self.FONT_MONO, width=7,
                     anchor='w').pack(side=tk.LEFT)
            var = tk.IntVar(value=default)
            val_lbl = tk.Label(row, text=f"{default:3d}", bg=self.PANEL_BG,
                               fg=self.ACCENT, font=("Courier New", 9),
                               width=3)
            tk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL,
                     variable=var, length=168,
                     bg=self.PANEL_BG, fg=self.TEXT,
                     troughcolor=self.BORDER, highlightthickness=0, bd=0,
                     sliderrelief=tk.FLAT, showvalue=False,
                     command=lambda v, a=attr, lbl=val_lbl: (
                         lbl.config(text=f"{int(v):3d}"),
                         self._update_hsv(a, int(v)))
                     ).pack(side=tk.LEFT)
            val_lbl.pack(side=tk.LEFT, padx=(4, 0))
            setattr(self, attr, var)

        # Pulsante reset valori di default (solo pallina)
        tk.Button(parent, text="↺ RESET DEFAULT",
                  bg=self.BORDER, fg=self.MUTED,
                  relief=tk.FLAT, font=("Courier New", 8),
                  cursor="hand2",
                  command=self._reset_hsv_defaults
                  ).pack(anchor='e', pady=(4, 0))

        # ── ROI margin ───────────────────────────────────────────────────── #
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=6)
        tk.Label(parent, text="ROI MARGIN",
                 bg=self.PANEL_BG, fg=self.YELLOW,
                 font=("Courier New", 9, "bold")).pack(anchor='w')

        roi_row = tk.Frame(parent, bg=self.PANEL_BG)
        roi_row.pack(fill=tk.X, pady=1)
        tk.Label(roi_row, text="Margin", bg=self.PANEL_BG, fg=self.MUTED,
                 font=self.FONT_MONO, width=7,
                 anchor='w').pack(side=tk.LEFT)
        self.roi_margin_var = tk.IntVar(value=ROI_RADIUS_MARGIN)
        roi_val_lbl = tk.Label(roi_row, text=f"{ROI_RADIUS_MARGIN:3d}",
                               bg=self.PANEL_BG, fg=self.YELLOW,
                               font=("Courier New", 9), width=3)
        tk.Scale(roi_row, from_=0, to=60, orient=tk.HORIZONTAL,
                 variable=self.roi_margin_var, length=168,
                 bg=self.PANEL_BG, fg=self.TEXT,
                 troughcolor=self.BORDER, highlightthickness=0, bd=0,
                 sliderrelief=tk.FLAT, showvalue=False,
                 command=lambda v, lbl=roi_val_lbl: (
                     lbl.config(text=f"{int(v):3d}"),
                     self._update_roi_margin(int(v)))
                 ).pack(side=tk.LEFT)
        roi_val_lbl.pack(side=tk.LEFT, padx=(4, 0))
        tk.Label(roi_row, text="px", bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 8)).pack(side=tk.LEFT, padx=(4, 0))

        # ── Preset HSV ──────────────────────────────────────────────────── #
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=6)
        tk.Label(parent, text="PRESET HSV", bg=self.PANEL_BG,
                 fg=self.MUTED, font=("Courier New", 9, "bold")
                 ).pack(anchor='w')

        preset_row = tk.Frame(parent, bg=self.PANEL_BG)
        preset_row.pack(fill=tk.X, pady=(4, 0))

        self._preset_btns = []
        for i in range(1, 4):                    # Preset 1, 2, 3
            col = tk.Frame(preset_row, bg=self.PANEL_BG)
            col.pack(side=tk.LEFT, padx=(0, 6))

            name_var = tk.StringVar(value=f"Preset {i}")
            name_entry = tk.Entry(col, textvariable=name_var, width=9,
                                  bg=self.BORDER, fg=self.TEXT,
                                  insertbackground=self.TEXT,
                                  relief=tk.FLAT,
                                  font=("Courier New", 8))
            name_entry.pack(fill=tk.X, pady=(0, 2))

            btn_load = tk.Button(col, text="▶ CARICA",
                                 bg=self.BORDER, fg=self.ACCENT,
                                 relief=tk.FLAT,
                                 font=("Courier New", 8, "bold"),
                                 cursor="hand2",
                                 command=lambda idx=i: self._load_preset(idx))
            btn_load.pack(fill=tk.X, pady=1)

            btn_save = tk.Button(col, text="⬇ SALVA",
                                 bg=self.BORDER, fg=self.YELLOW,
                                 relief=tk.FLAT,
                                 font=("Courier New", 8),
                                 cursor="hand2",
                                 command=lambda idx=i, nv=name_var:
                                     self._save_preset(idx, nv.get()))
            btn_save.pack(fill=tk.X, pady=1)

            # Etichetta info preset (mostra cosa contiene)
            info_lbl = tk.Label(col, text="vuoto",
                                bg=self.PANEL_BG, fg=self.MUTED,
                                font=("Courier New", 7), wraplength=80)
            info_lbl.pack(anchor='w')

            self._preset_btns.append({
                'name_var':  name_var,
                'btn_load':  btn_load,
                'info_lbl':  info_lbl,
            })

        # Inizializza dizionario preset (slot 1-3) — carica da file se esiste
        self._hsv_presets = {1: None, 2: None, 3: None}
        self._refresh_preset_ui()

    def _update_hsv(self, attr, val):
        global LOWER_BALL, UPPER_BALL
        m = {
            'hsv_ball_h_min': (LOWER_BALL, 0),
            'hsv_ball_h_max': (UPPER_BALL, 0),
            'hsv_ball_s_min': (LOWER_BALL, 1),
            'hsv_ball_v_min': (LOWER_BALL, 2),
        }
        if attr in m:
            m[attr][0][m[attr][1]] = val

    def _update_roi_margin(self, val: int):
        global ROI_RADIUS_MARGIN
        ROI_RADIUS_MARGIN = val

    def _reset_hsv_defaults(self):
        global LOWER_BALL, UPPER_BALL, ROI_RADIUS_MARGIN
        defaults = {
            'hsv_ball_h_min': 5,  'hsv_ball_h_max': 35,
            'hsv_ball_s_min': 80, 'hsv_ball_v_min': 120,
        }
        for attr, val in defaults.items():
            getattr(self, attr).set(val)
            self._update_hsv(attr, val)
        LOWER_BALL = np.array([5,  80, 120])
        UPPER_BALL = np.array([35, 255, 255])
        ROI_RADIUS_MARGIN = 18
        self.roi_margin_var.set(18)

    # ── Metodi preset HSV ───────────────────────────────────────────────── #

    def _current_hsv_values(self) -> dict:
        keys = ['hsv_ball_h_min', 'hsv_ball_h_max',
                'hsv_ball_s_min', 'hsv_ball_v_min']
        d = {k: getattr(self, k).get() for k in keys}
        d['roi_margin'] = self.roi_margin_var.get()
        return d

    def _apply_hsv_values(self, values: dict):
        global LOWER_BALL, UPPER_BALL, ROI_RADIUS_MARGIN
        for attr in ('hsv_ball_h_min', 'hsv_ball_h_max',
                     'hsv_ball_s_min', 'hsv_ball_v_min'):
            if attr in values and hasattr(self, attr):
                getattr(self, attr).set(values[attr])
                self._update_hsv(attr, values[attr])
        LOWER_BALL = np.array([values.get('hsv_ball_h_min', 5),
                                values.get('hsv_ball_s_min', 80),
                                values.get('hsv_ball_v_min', 120)])
        UPPER_BALL = np.array([values.get('hsv_ball_h_max', 35), 255, 255])
        if 'roi_margin' in values:
            ROI_RADIUS_MARGIN = int(values['roi_margin'])
            self.roi_margin_var.set(ROI_RADIUS_MARGIN)

    def _save_preset(self, idx: int, name: str):
        """Salva i valori correnti nello slot idx e scrive su file JSON."""
        self._hsv_presets[idx] = {
            'name':   name.strip() or f"Preset {idx}",
            'values': self._current_hsv_values(),
        }
        self._refresh_preset_ui()
        self._save_presets_to_file()
        self._log(f"Preset {idx} salvato: '{self._hsv_presets[idx]['name']}'",
                  "system")

    def _load_preset(self, idx: int):
        """Carica nello slot idx e aggiorna slider + array."""
        p = self._hsv_presets.get(idx)
        if p is None:
            self._log(f"Preset {idx} vuoto — prima salva qualcosa.", "error")
            return
        self._apply_hsv_values(p['values'])
        self._log(f"Preset {idx} caricato: '{p['name']}'", "system")

    def _save_presets_to_file(self):
        """Serializza _hsv_presets in JSON su PRESETS_FILE."""
        try:
            # Converti le chiavi intere in stringa (JSON richiede chiavi stringa)
            data = {str(k): v for k, v in self._hsv_presets.items()}
            PRESETS_FILE.write_text(
                json.dumps(data, indent=2, ensure_ascii=False),
                encoding="utf-8"
            )
            self._log(f"Preset salvati in {PRESETS_FILE.name}", "system")
        except OSError as e:
            self._log(f"Errore scrittura preset: {e}", "error")

    def _load_presets_from_file(self):
        """Carica i preset da PRESETS_FILE se esiste, silenzioso se assente."""
        if not PRESETS_FILE.exists():
            return
        try:
            raw  = json.loads(PRESETS_FILE.read_text(encoding="utf-8"))
            # Riconverti le chiavi stringa in interi
            for k, v in raw.items():
                slot = int(k)
                if slot in self._hsv_presets and v is not None:
                    self._hsv_presets[slot] = v
            self._log(
                f"Preset caricati da {PRESETS_FILE.name} "
                f"({sum(1 for v in self._hsv_presets.values() if v)} slot occupati)",
                "system"
            )
        except (OSError, json.JSONDecodeError, ValueError) as e:
            self._log(f"Errore lettura preset: {e}", "error")

    def _refresh_preset_ui(self):
        """Aggiorna etichette info e colore pulsante CARICA per ogni slot."""
        for i, entry in enumerate(self._preset_btns, start=1):
            p = self._hsv_presets.get(i)
            if p is None:
                entry['info_lbl'].config(text="vuoto", fg=self.MUTED)
                entry['btn_load'].config(fg=self.MUTED)
            else:
                v = p['values']
                summary = (
                    f"H {v.get('hsv_ball_h_min','?')}-{v.get('hsv_ball_h_max','?')} "
                    f"S {v.get('hsv_ball_s_min','?')} "
                    f"V {v.get('hsv_ball_v_min','?')}"
                )
                entry['name_var'].set(p['name'])
                entry['info_lbl'].config(text=summary, fg=self.ACCENT)
                entry['btn_load'].config(fg=self.ACCENT)

    # ══════════════════════════════════════════════════════════════════════ #
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
    #  TAB ARUCO — Calibrazione marker piattaforma
    # ══════════════════════════════════════════════════════════════════════ #

    def _build_aruco_tab(self, parent):
        # ── Stato calibrazione ───────────────────────────────────────────── #
        self._section_label(parent, "STATO CALIBRAZIONE")

        self.lbl_calib_status = tk.Label(
            parent,
            text="● NON CALIBRATO",
            bg=self.PANEL_BG, fg=self.RED,
            font=("Courier New", 10, "bold"))
        self.lbl_calib_status.pack(anchor='w', pady=(4, 0))

        self.lbl_calib_detail = tk.Label(
            parent, text="Nessun file di calibrazione trovato.",
            bg=self.PANEL_BG, fg=self.MUTED,
            font=("Courier New", 8), wraplength=340, justify=tk.LEFT)
        self.lbl_calib_detail.pack(anchor='w', pady=(2, 6))

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=6)

        # ── ID marker ────────────────────────────────────────────────────── #
        self._section_label(parent, "ID MARKER (devono corrispondere ai marker stampati)")

        id_row = tk.Frame(parent, bg=self.PANEL_BG)
        id_row.pack(fill=tk.X, pady=(4, 8))

        self._marker_id_vars = []
        for i, default_id in enumerate(ARUCO_IDS_DEFAULT):
            tk.Label(id_row, text=f"M{i}:", bg=self.PANEL_BG, fg=self.MUTED,
                     font=self.FONT_MONO).pack(side=tk.LEFT, padx=(0 if i == 0 else 8, 2))
            var = tk.IntVar(value=default_id)
            tk.Spinbox(id_row, from_=0, to=49, textvariable=var, width=3,
                       bg=self.BORDER, fg=self.TEXT,
                       buttonbackground=self.BORDER, relief=tk.FLAT,
                       font=self.FONT_MONO).pack(side=tk.LEFT)
            self._marker_id_vars.append(var)

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=6)

        # ── Pulsante calibrazione ─────────────────────────────────────────── #
        self._section_label(parent, "PROCEDURA")
        tk.Label(parent,
                 text="1. Avvia la camera\n"
                      "2. Assicurati che tutti e 4 i marker siano visibili\n"
                      "3. Tieni la piattaforma ferma e orizzontale\n"
                      "4. Premi CALIBRA",
                 bg=self.PANEL_BG, fg=self.MUTED,
                 font=("Courier New", 8), justify=tk.LEFT).pack(anchor='w',
                                                                 pady=(4, 8))

        self.btn_calibrate = tk.Button(
            parent, text="🎯  CALIBRA ORA",
            bg=self.ACCENT, fg=self.DARK_BG,
            relief=tk.FLAT,
            font=("Courier New", 10, "bold"),
            cursor="hand2", padx=12, pady=7,
            command=self._do_calibrate)
        self.btn_calibrate.pack(fill=tk.X)

        tk.Button(parent, text="✕  RESET CALIBRAZIONE",
                  bg=self.BORDER, fg=self.RED,
                  relief=tk.FLAT, font=("Courier New", 9),
                  cursor="hand2", pady=4,
                  command=self._reset_aruco_calib
                  ).pack(fill=tk.X, pady=(6, 0))

        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill=tk.X, pady=8)

        # ── Stato marker live ─────────────────────────────────────────────── #
        self._section_label(parent, "MARKER VISIBILI (live)")
        self._marker_leds = []
        led_row = tk.Frame(parent, bg=self.PANEL_BG)
        led_row.pack(fill=tk.X, pady=(4, 0))
        for i in range(4):
            col = tk.Frame(led_row, bg=self.PANEL_BG)
            col.pack(side=tk.LEFT, padx=(0, 16))
            lbl = tk.Label(col, text=f"● M{i}\nID ?",
                           bg=self.PANEL_BG, fg=self.MUTED,
                           font=("Courier New", 9), justify=tk.CENTER)
            lbl.pack()
            self._marker_leds.append(lbl)

    # ── Azioni calibrazione ──────────────────────────────────────────────── #

    def _do_calibrate(self):
        """
        Cattura un singolo frame e tenta la rilevazione dei 4 marker.
        Bastano almeno 3 marker visibili per calibrare.
        """
        if not (self.vision_thread and self.vision_thread.is_alive()):
            messagebox.showwarning("Attenzione", "Avvia prima la camera.")
            return

        target_ids = [v.get() for v in self._marker_id_vars]
        if len(set(target_ids)) != 4:
            messagebox.showerror("Errore", "I 4 ID marker devono essere tutti diversi.")
            return

        # Richiedi un frame fresco alla camera
        # Svuotiamo la queue per avere l'ultimo frame disponibile
        last_frame = None
        for _ in range(5):
            time.sleep(0.06)
            try:
                while not self.frame_q.empty():
                    last_frame, _ = self.frame_q.get_nowait()
            except Exception:
                pass
            if last_frame is not None:
                break

        if last_frame is None:
            messagebox.showerror("Errore", "Nessun frame disponibile dalla camera.")
            return

        # Rileva marker ArUco sul frame catturato
        gray     = cv2.cvtColor(last_frame, cv2.COLOR_BGR2GRAY)
        aruco_d  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        aruco_p  = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_d, aruco_p)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            messagebox.showerror("Calibrazione fallita",
                                 "Nessun marker ArUco rilevato.\n"
                                 "Controlla che i marker siano visibili e ben illuminati.")
            return

        found = {}
        for i, mid in enumerate(ids.flatten()):
            if mid in target_ids:
                found[int(mid)] = corners[i][0].mean(axis=0).tolist()

        # Bastano 3 marker su 4 per calibrare
        if len(found) < ARUCO_MIN_VISIBLE:
            missing = [tid for tid in target_ids if tid not in found]
            messagebox.showerror(
                "Calibrazione fallita",
                f"Trovati solo {len(found)}/4 marker (servono almeno {ARUCO_MIN_VISIBLE}).\n"
                f"Marker mancanti: {missing}\n\n"
                "Verifica che i marker siano visibili e gli ID corretti.")
            return

        # Calcola circocentro come verifica
        # Kåsa circle fit su tutti i marker trovati — robusto al
        # posizionamento impreciso (minimizza somma quadrati distanze)
        center, radius = kasa_circle_fit(list(found.values()))
        if center is None or radius < 20:
            messagebox.showerror("Calibrazione fallita",
                                 "I marker sembrano collineari o troppo vicini.\n"
                                 "Controlla il posizionamento.")
            return

        # Per ogni marker salva l'offset rispetto al centro calibrato.
        # A runtime: centro_stimato = pos_rilevata - offset
        marker_offsets = {
            mid: (np.array(pos) - center).tolist()
            for mid, pos in found.items()
        }

        calib = {
            'marker_ids':       target_ids,
            'marker_centers':   found,                   # {id: [x, y]}
            'circle_center_px': center.tolist(),
            'circle_radius_px': round(radius, 1),
            'marker_offsets':   {str(k): v for k, v in marker_offsets.items()},
        }
        self.calib_ref[0] = calib
        self._save_aruco_calib(calib)
        self._update_calib_ui(calib)
        self._log(f"ArUco calibrato (Kåsa fit) — {len(found)}/4 marker, IDs {target_ids}, "
                  f"centro ({center[0]:.0f},{center[1]:.0f}), "
                  f"r={radius:.0f}px", "system")
        messagebox.showinfo("Calibrazione OK",
                            f"Piattaforma calibrata con successo!\n"
                            f"Metodo: Kåsa least-squares fit\n"
                            f"Marker usati: {len(found)}/4\n"
                            f"Centro: ({center[0]:.0f}, {center[1]:.0f}) px\n"
                            f"Raggio: {radius:.0f} px\n\n"
                            f"Il posizionamento impreciso dei marker\n"
                            f"è stato compensato automaticamente.")

    def _reset_aruco_calib(self):
        if not messagebox.askyesno("Reset calibrazione",
                                   "Eliminare la calibrazione corrente?"):
            return
        self.calib_ref[0] = None
        if ARUCO_CALIB_FILE.exists():
            ARUCO_CALIB_FILE.unlink()
        self.lbl_calib_status.config(text="● NON CALIBRATO", fg=self.RED)
        self.lbl_calib_detail.config(text="Calibrazione eliminata.")
        self._log("Calibrazione ArUco eliminata.", "system")

    def _save_aruco_calib(self, calib: dict):
        try:
            ARUCO_CALIB_FILE.write_text(
                json.dumps(calib, indent=2), encoding="utf-8")
        except OSError as e:
            self._log(f"Errore scrittura calibrazione: {e}", "error")

    def _load_aruco_calib(self):
        if not ARUCO_CALIB_FILE.exists():
            return
        try:
            calib = json.loads(ARUCO_CALIB_FILE.read_text(encoding="utf-8"))
            # Normalizza chiavi int (JSON usa stringhe)
            # Converti chiavi stringa in int (JSON le serializza come str)
            for key in ('marker_centers', 'marker_offsets'):
                if key in calib:
                    calib[key] = {int(k): v for k, v in calib[key].items()}
            self.calib_ref[0] = calib
            self._update_calib_ui(calib)
            self._log(f"Calibrazione ArUco caricata — IDs {calib['marker_ids']}", "system")
        except (OSError, json.JSONDecodeError, KeyError) as e:
            self._log(f"Errore lettura calibrazione ArUco: {e}", "error")

    def _update_calib_ui(self, calib: dict):
        """Aggiorna le etichette nel tab ArUco con i dati della calibrazione."""
        ids = calib['marker_ids']
        cx, cy = calib.get('circle_center_px', calib.get('circumcenter_px', [0,0]))
        r  = calib.get('circle_radius_px', calib.get('circumradius_px', 0))
        n_off = len(calib.get('marker_offsets', {}))
        self.lbl_calib_status.config(
            text=f"● CALIBRATO  IDs: {ids}", fg=self.ACCENT)
        self.lbl_calib_detail.config(
            text=f"Centro: ({cx:.0f}, {cy:.0f})  |  Raggio: {r:.0f} px  |  "
                 f"Offset: {n_off}/{len(ids)} marker\n"
                 f"File: {ARUCO_CALIB_FILE.name}")
        # Aggiorna anche gli spinbox ID
        for var, mid in zip(self._marker_id_vars, ids):
            var.set(mid)

    # ── Poll marker LED (chiamato da _poll_info) ─────────────────────────── #

    def _update_marker_leds(self, markers_seen: list):
        calib = self.calib_ref[0]
        target_ids = calib['marker_ids'] if calib else ARUCO_IDS_DEFAULT
        for i, (lbl, mid) in enumerate(zip(self._marker_leds, target_ids)):
            seen = mid in markers_seen
            color = self.ACCENT if seen else self.RED
            lbl.config(text=f"{'●' if seen else '○'} M{i}\nID {mid}", fg=color)

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
                self.ser_ref, self.run_ev, self.calib_ref)
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
                    self._update_marker_leds(info.get('markers', []))
                elif info['type'] == 'status':
                    # Aggiorna LED marker e stato piattaforma/pallina senza loggare
                    self._update_marker_leds(info.get('markers', []))
                    pc = self.ACCENT if info['plat'] else self.MUTED
                    bc = self.ACCENT if info['ball'] else self.MUTED
                    self.lbl_plat.config(
                        text=f"● Piattaforma: {'OK' if info['plat'] else 'N/D'}",
                        fg=pc)
                    self.lbl_ball.config(
                        text=f"● Pallina:      {'OK' if info['ball'] else 'N/D'}",
                        fg=bc)
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