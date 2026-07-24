#!/usr/bin/env python3
"""
LEGO YOLOv11 Detection Tester
Raspberry Pi GUI for testing NCNN-converted YOLOv11 models.
"""

import serial
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time
import os
import subprocess
from pathlib import Path
from PIL import Image, ImageTk, ImageDraw, ImageFont
import cv2
import numpy as np

# Try importing ultralytics for NCNN export
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

# ── Colours & style ───────────────────────────────────────────────────────────
BG       = "#0f1117"
PANEL_BG = "#1a1d27"
ACCENT   = "#f5c518"   # LEGO yellow
ACCENT2  = "#e84040"   # detection red
TEXT     = "#e8eaf0"
TEXT_DIM = "#6b7280"
BORDER   = "#2d3148"
BTN_BG   = "#252840"
BTN_HOV  = "#353a5e"
FONT_MONO = ("Courier New", 10)
FONT_UI   = ("Helvetica", 10)
FONT_HEAD = ("Helvetica", 13, "bold")

ALLOWED_CLASSES = {"1x1 Bricks", "1x2 Bricks", "1x4 Bricks", "2x2 Bricks", "2x4 Bricks"}


def draw_detections(pil_img, results):
    """Draw bounding boxes + labels on a PIL image from ultralytics Results."""
    draw = ImageDraw.Draw(pil_img)
    try:
        font    = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
        font_sm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
    except Exception:
        font    = ImageFont.load_default()
        font_sm = font

    if results is None:
        return pil_img

    for r in results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf   = float(box.conf[0])
            cls_id = int(box.cls[0])
            label  = r.names.get(cls_id, str(cls_id)) if r.names else str(cls_id)
            tag    = f"{label} {conf:.2f}"

            draw.rectangle([x1, y1, x2, y2], outline=ACCENT2, width=2)
            tw, th = draw.textsize(tag, font=font) if hasattr(draw, "textsize") else (len(tag) * 8, 16)
            draw.rectangle([x1, y1 - th - 4, x1 + tw + 6, y1], fill=ACCENT2)
            draw.text((x1 + 3, y1 - th - 2), tag, fill="white", font=font)

    return pil_img


# ── Serial controller ─────────────────────────────────────────────────────────
class SerialController:
    def __init__(self, on_line_cb, on_status_cb):
        self.on_line_cb   = on_line_cb
        self.on_status_cb = on_status_cb
        self.ser    = None
        self.thread = None
        self.alive  = False
        self.rx_queue = queue.Queue()

    def connect(self, port="/dev/ttyAMA0", baud=115200):
        self.disconnect()
        self.ser   = serial.Serial(port, baudrate=baud, timeout=0.1)
        self.alive = True
        self.thread = threading.Thread(target=self._rx_thread, daemon=True)
        self.thread.start()
        self.on_status_cb(f"Connected: {port} @ {baud}")

    def disconnect(self):
        self.alive = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.on_status_cb("Disconnected")

    def send_line(self, line: str):
        if not self.ser:
            return
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode("utf-8", errors="replace"))

    def _rx_thread(self):
        try:
            while self.alive and self.ser:
                line = self.ser.readline()
                if not line:
                    continue
                try:
                    s = line.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                self.rx_queue.put(s)
        except Exception as e:
            self.rx_queue.put(f"__ERR__ {e}")

    def poll(self):
        """Call from Tk thread periodically to drain rx_queue."""
        drained = 0
        while True:
            try:
                s = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self.on_line_cb(s)
        return drained


# ── Main App ──────────────────────────────────────────────────────────────────
class LegoDetectorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LEGO YOLOv11 — Detection Tester")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(960, 560)

        # State
        self.model           = None
        self.ncnn_model_dir  = None
        self.cap             = None
        self.live_running    = False
        self.capture_thread  = None
        self.last_frame      = None   # raw numpy frame
        self.inference_image = None   # PIL image with detections

        # UART state
        self.serial_port_var   = tk.StringVar(value="/dev/ttyAMA0")
        self.serial_status_var = tk.StringVar(value="Disconnected")

        self._inference_busy  = False
        self._inference_ran   = False  # True once any *non-empty* inference has completed
        self._last_class_name = None   # class name from most recent non-empty inference
        self._last_result_ok  = False  # True only when exactly one allowed class detected

        self.serctl = SerialController(
            on_line_cb=self._on_serial_line,
            on_status_cb=lambda t: self.serial_status_var.set(t),
        )

        self._build_ui()
        self._start_camera()
        self.after(30, self._poll_serial)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # Top bar
        topbar = tk.Frame(self, bg=BG, pady=8, padx=14)
        topbar.pack(fill="x", side="top")

        tk.Label(topbar, text="⬛ LEGO DETECTOR", font=("Helvetica", 15, "bold"),
                 bg=BG, fg=ACCENT).pack(side="left")

        ctrl = tk.Frame(topbar, bg=BG)
        ctrl.pack(side="right")

        self.model_path_var = tk.StringVar(value="No model loaded")
        tk.Label(ctrl, textvariable=self.model_path_var, bg=BG, fg=TEXT_DIM,
                 font=FONT_MONO, width=40, anchor="e").pack(side="left", padx=(0, 8))

        self._btn(ctrl, "📂 Load Model (.pt / NCNN dir)", self._load_model).pack(side="left", padx=2)

        self.model_status = tk.Label(ctrl, text="● No model", bg=BG, fg=ACCENT2, font=FONT_UI)
        self.model_status.pack(side="left", padx=8)

        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")

        # UART bar
        uartbar = tk.Frame(self, bg=BG, padx=14, pady=6)
        uartbar.pack(fill="x")

        tk.Label(uartbar, text="UART:", bg=BG, fg=TEXT_DIM, font=FONT_UI).pack(side="left")
        tk.Entry(uartbar, textvariable=self.serial_port_var, width=16,
                 bg=PANEL_BG, fg=TEXT, insertbackground=TEXT, relief="flat").pack(side="left", padx=(6, 6))

        self._btn(uartbar, "Connect",    self._uart_connect).pack(side="left", padx=2)
        self._btn(uartbar, "Disconnect", self._uart_disconnect).pack(side="left", padx=2)
        self._btn(uartbar, "Ping", lambda: self._uart_send("PING")).pack(side="left", padx=6)

        tk.Label(uartbar, text="Send:", bg=BG, fg=TEXT_DIM, font=FONT_UI).pack(side="left", padx=(10, 0))
        self.raw_cmd_var = tk.StringVar(value="")
        tk.Entry(uartbar, textvariable=self.raw_cmd_var, width=24,
                 bg=PANEL_BG, fg=TEXT, insertbackground=TEXT, relief="flat").pack(side="left", padx=2)
        self._btn(uartbar, "Send", self._send_raw_cmd).pack(side="left", padx=2)

        tk.Label(uartbar, textvariable=self.serial_status_var, bg=BG, fg=TEXT_DIM,
                 font=FONT_MONO, anchor="e").pack(side="right")

        # Main panels
        main = tk.Frame(self, bg=BG)
        main.pack(fill="both", expand=True, padx=14, pady=10)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # Left panel – live feed
        left = tk.Frame(main, bg=PANEL_BG, bd=0, highlightthickness=1, highlightbackground=BORDER)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 7))

        tk.Label(left, text="LIVE FEED", font=FONT_HEAD, bg=PANEL_BG, fg=ACCENT, pady=6).pack(fill="x")
        tk.Frame(left, bg=BORDER, height=1).pack(fill="x")

        self.live_canvas = tk.Canvas(left, bg="#0a0c12", cursor="crosshair", highlightthickness=0)
        self.live_canvas.pack(fill="both", expand=True, padx=8, pady=8)

        self.cam_status_var = tk.StringVar(value="Initialising camera…")
        tk.Label(left, textvariable=self.cam_status_var, bg=PANEL_BG, fg=TEXT_DIM,
                 font=FONT_MONO, anchor="w", padx=8).pack(fill="x", pady=(0, 4))

        # Right panel – inference view
        right = tk.Frame(main, bg=PANEL_BG, bd=0, highlightthickness=1, highlightbackground=BORDER)
        right.grid(row=0, column=1, sticky="nsew", padx=(7, 0))

        tk.Label(right, text="INFERENCE", font=FONT_HEAD, bg=PANEL_BG, fg=ACCENT, pady=6).pack(fill="x")
        tk.Frame(right, bg=BORDER, height=1).pack(fill="x")

        self.infer_canvas = tk.Canvas(right, bg="#0a0c12", highlightthickness=0)
        self.infer_canvas.pack(fill="both", expand=True, padx=8, pady=8)

        self.stats_var = tk.StringVar(value="No inference yet")
        tk.Label(right, textvariable=self.stats_var, bg=PANEL_BG, fg=TEXT_DIM,
                 font=FONT_MONO, anchor="w", padx=8).pack(fill="x")

        btn_frame = tk.Frame(right, bg=PANEL_BG, pady=8)
        btn_frame.pack(fill="x", padx=8)

        self.capture_btn = self._btn(btn_frame, "📸  CAPTURE & RUN INFERENCE",
                                     self._capture_and_infer, big=True)
        self.capture_btn.pack(fill="x")

        self.send_btn = self._btn(btn_frame, "📡  SEND TO STM32",
                                  self._send_to_stm32, big=True)
        self.send_btn.pack(fill="x", pady=(6, 0))

        # Log strip
        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")
        log_frame = tk.Frame(self, bg=BG, height=80)
        log_frame.pack(fill="x", padx=14, pady=(4, 8))
        log_frame.pack_propagate(False)

        tk.Label(log_frame, text="LOG", font=("Helvetica", 9, "bold"),
                 bg=BG, fg=TEXT_DIM).pack(anchor="w")

        self.log_text = tk.Text(log_frame, bg=BG, fg=TEXT_DIM, font=FONT_MONO,
                                height=4, bd=0, state="disabled", wrap="word",
                                insertbackground=ACCENT)
        self.log_text.pack(fill="both", expand=True)

    def _btn(self, parent, text, cmd, big=False):
        size = 12 if big else 10
        b = tk.Button(parent, text=text, command=cmd,
                      bg=BTN_BG, fg=TEXT, activebackground=BTN_HOV,
                      activeforeground=ACCENT, relief="flat",
                      font=("Helvetica", size, "bold" if big else "normal"),
                      cursor="hand2", pady=6 if big else 3, padx=10,
                      bd=0, highlightthickness=0)
        b.bind("<Enter>", lambda e: b.config(bg=BTN_HOV))
        b.bind("<Leave>", lambda e: b.config(bg=BTN_BG))
        return b

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ── Camera ────────────────────────────────────────────────────────────────

    CAM_W, CAM_H = 640, 480

    def _start_camera(self):
        self.live_running    = True
        self._cam_frame_lock = threading.Lock()
        self._cam_proc       = None
        threading.Thread(target=self._camera_reader_thread, daemon=True).start()
        self._poll_camera()

    def _camera_reader_thread(self):
        w, h = self.CAM_W, self.CAM_H
        cmd = [
            "rpicam-vid",
            "--width",     str(w),
            "--height",    str(h),
            "--framerate", "25",
            "--codec",     "yuv420",
            "--timeout",   "0",
            "--nopreview",
            "-o", "-",
        ]
        self._log(f"Starting: {' '.join(cmd)}")
        self.after(0, lambda: self.cam_status_var.set("Starting rpicam-vid…"))

        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
            self._cam_proc = proc
        except FileNotFoundError:
            self._log("ERROR: rpicam-vid not found. Is rpicam-apps installed?")
            self.after(0, lambda: self.cam_status_var.set("⚠ rpicam-vid not found"))
            return

        yuv_bytes = w * h * 3 // 2
        self.after(0, lambda: self.cam_status_var.set(f"rpicam-vid  {w}×{h}  ▶ running"))
        self._log("Camera stream started.")

        buf = b""
        while self.live_running:
            try:
                chunk = proc.stdout.read(yuv_bytes - len(buf))
                if not chunk:
                    break
                buf += chunk
                if len(buf) < yuv_bytes:
                    continue

                raw = np.frombuffer(buf[:yuv_bytes], dtype=np.uint8)
                buf = buf[yuv_bytes:]
                yuv = raw.reshape((h * 3 // 2, w))
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

                with self._cam_frame_lock:
                    self.last_frame = bgr

            except Exception as e:
                if self.live_running:
                    self._log(f"Camera read error: {e}")
                break

        proc.terminate()
        self._log("Camera stream stopped.")

    def _poll_camera(self):
        if not self.live_running:
            return

        with self._cam_frame_lock:
            frame = self.last_frame.copy() if self.last_frame is not None else None

        if frame is not None:
            img   = self._cv2_to_pil(frame)
            img   = self._fit_to_canvas(img, self.live_canvas)
            imgtk = ImageTk.PhotoImage(img)
            self._draw_on_canvas(self.live_canvas, imgtk)
        else:
            ph_img = self._placeholder_canvas(self.live_canvas, "No Camera Signal")
            self._draw_on_canvas(self.live_canvas, ph_img)

        self.after(40, self._poll_camera)

    # ── Model loading ─────────────────────────────────────────────────────────

    def _load_model(self):
        path = filedialog.askopenfilename(
            title="Select YOLOv11 model",
            filetypes=[("YOLO / NCNN models", "*.pt *.yaml *.bin"), ("All files", "*.*")]
        )
        if not path:
            return

        ext = Path(path).suffix.lower()
        if ext in (".bin", ".param"):
            self._load_ncnn_dir(str(Path(path).parent))
            return
        if ext == ".pt":
            self._convert_and_load(path)
        else:
            messagebox.showerror("Unsupported", "Please select a .pt file or an NCNN directory.")

    def _load_ncnn_dir(self, directory):
        if not ULTRALYTICS_AVAILABLE:
            messagebox.showerror("Missing dependency", "ultralytics is not installed.")
            return
        self._log(f"Loading NCNN model from: {directory}")
        self.model_status.config(text="● Loading…", fg=ACCENT)
        self.model_path_var.set(os.path.basename(directory))

        def _load():
            try:
                model = (YOLO(os.path.join(directory, "model.yaml"))
                         if os.path.exists(os.path.join(directory, "model.yaml"))
                         else YOLO(directory))
                self.model          = model
                self.ncnn_model_dir = directory
                self.after(0, lambda: self.model_status.config(text="● NCNN ready", fg="#4ade80"))
                self._log("NCNN model loaded successfully.")
            except Exception as e:
                self.after(0, lambda: self.model_status.config(text="● Load error", fg=ACCENT2))
                self._log(f"ERROR loading NCNN model: {e}")

        threading.Thread(target=_load, daemon=True).start()

    def _convert_and_load(self, pt_path):
        if not ULTRALYTICS_AVAILABLE:
            messagebox.showerror("Missing dependency",
                                 "ultralytics package is required.\nInstall with: pip install ultralytics")
            return

        self._log(f"Converting {os.path.basename(pt_path)} → NCNN …")
        self.model_status.config(text="● Converting…", fg=ACCENT)
        self.model_path_var.set(os.path.basename(pt_path))

        def _do_convert():
            try:
                model       = YOLO(pt_path)
                export_path = model.export(format="ncnn")
                self._log(f"Exported to: {export_path}")
                ncnn_model          = YOLO(export_path)
                self.model          = ncnn_model
                self.ncnn_model_dir = export_path
                self.after(0, lambda: self.model_status.config(text="● NCNN ready", fg="#4ade80"))
                self._log("Model converted and loaded. Ready for inference.")
            except Exception as e:
                self.after(0, lambda: self.model_status.config(text="● Error", fg=ACCENT2))
                self._log(f"ERROR during conversion: {e}")

        threading.Thread(target=_do_convert, daemon=True).start()

    # ── Shared inference helper ───────────────────────────────────────────────

    def _run_inference(self, frame):
        """
        Run YOLO on `frame` and return (results, dets, elapsed_ms).
        Called from a background thread.

        IMPORTANT:
        - If no pieces are detected at all (dets is empty), this case is treated
          as "nothing to report" and is IGNORED — the previously stored result
          state (_last_class_name / _last_result_ok / _inference_ran) is left
          untouched, so an empty frame can never overwrite a real, previous result.
        - Multiple detections of the SAME class are fine — they still resolve to
          that one class (e.g. three "2x2 Bricks" boxes → "2x2 Bricks"). Only a
          MIX of different classes in one frame is treated as ambiguous/invalid.
        """
        t0      = time.perf_counter()
        results = self.model(frame, verbose=False)
        elapsed = (time.perf_counter() - t0) * 1000

        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                name   = r.names.get(cls_id, str(cls_id)) if r.names else str(cls_id)
                dets.append(name)

        if len(dets) == 0:
            # Nothing detected — ignore this run entirely for state-tracking purposes.
            return results, dets, elapsed

        unique_classes = set(dets)
        if len(unique_classes) == 1:
            # One or more boxes, all the same class — resolve to that class.
            cls = dets[0]
            self._last_class_name = cls
            self._last_result_ok  = cls in ALLOWED_CLASSES
        else:
            # Mixed classes in the same frame — ambiguous.
            self._last_class_name = None
            self._last_result_ok  = False

        self._inference_ran = True   # mark that at least one non-empty inference has completed

        return results, dets, elapsed

    # ── Manual capture & inference ────────────────────────────────────────────

    def _capture_and_infer(self):
        if self.model is None:
            messagebox.showwarning("No model", "Please load a model first.")
            return
        if self.last_frame is None:
            messagebox.showwarning("No frame", "No camera frame available yet.")
            return

        frame = self.last_frame.copy()
        self.capture_btn.config(state="disabled", text="⏳  Running inference…")
        self._log("Inference started…")

        def _infer():
            try:
                results, dets, elapsed = self._run_inference(frame)

                pil_img = self._cv2_to_pil(frame)
                pil_img = draw_detections(pil_img, results)
                self.inference_image = pil_img

                if len(dets) == 0:
                    # No pieces detected — ignore, don't treat as a result.
                    stats = "No pieces detected — ignored"
                    self._log("No pieces detected — ignoring this capture.")
                else:
                    stats = f"Detections: {len(dets)}   |   Inference: {elapsed:.1f} ms"
                    self._log(f"Done – {stats}")

                self.after(0, lambda: self._show_inference(pil_img, stats))

            except Exception as e:
                self._last_class_name = None
                self._last_result_ok  = False
                self._log(f"ERROR during inference: {e}")
            finally:
                self.after(0, lambda: self.capture_btn.config(
                    state="normal", text="📸  CAPTURE & RUN INFERENCE"))

        threading.Thread(target=_infer, daemon=True).start()

    def _show_inference(self, pil_img, stats):
        img   = self._fit_to_canvas(pil_img, self.infer_canvas)
        imgtk = ImageTk.PhotoImage(img)
        self._draw_on_canvas(self.infer_canvas, imgtk)
        self.stats_var.set(stats)

    # ── Send last result to STM32 ─────────────────────────────────────────────

    def _send_to_stm32(self):
        # Only block if no (non-empty) inference has been run at all yet
        if not self._inference_ran:
            messagebox.showwarning("No result", "Run inference first.")
            return

        # Valid detection → send class name; everything else → ERR
        cmd = self._last_class_name if self._last_result_ok else "ERR"
        self._uart_send(cmd)
        self._log(f"STM32 ← '{cmd}'")

    # ── Auto-classify pipeline (triggered by STM32 CLASSIFY request) ──────────

    def _on_serial_line(self, s: str):
        if s.startswith("__ERR__"):
            self._log(f"UART error: {s}")
            self.serial_status_var.set("UART error")
            return

        self._log(f"RX: {s}")

        parts = s.split()
        if not parts:
            return

        if parts[0] == "CLASSIFY" and len(parts) >= 2:
            self._handle_classify_request(parts[1])
        elif parts[0] == "CMRA":
            self._handle_classify_request("auto")

    def _handle_classify_request(self, req_id: str):
        if self.model is None:
            self._uart_send(f"RESULT {req_id} ERR FAIL NO_MODEL")
            return
        if self._inference_busy:
            self._uart_send(f"RESULT {req_id} ERR FAIL BUSY")
            return

        with self._cam_frame_lock:
            frame = self.last_frame.copy() if self.last_frame is not None else None

        if frame is None:
            self._uart_send(f"RESULT {req_id} ERR FAIL NO_FRAME")
            return

        self._inference_busy = True
        self._log(f"Auto inference for req_id={req_id}…")

        def _infer():
            try:
                results, dets, elapsed = self._run_inference(frame)

                # Update inference panel regardless of outcome
                pil_img = self._cv2_to_pil(frame)
                pil_img = draw_detections(pil_img, results)

                if len(dets) == 0:
                    # No pieces detected at all — ignore this request entirely.
                    # No reply is sent to the STM32; it should just retry / wait
                    # for a frame that actually has a piece in view.
                    stats = f"Auto req {req_id} | no pieces detected — ignored"
                    self.after(0, lambda: self._show_inference(pil_img, stats))
                    self._log(f"No pieces detected for req_id={req_id} — ignoring (no reply sent).")
                    return

                unique_classes = set(dets)

                if req_id == "auto":
                    # STM32-style direct command.
                    # Multiple boxes of the SAME class still resolve to that bin;
                    # only a mix of different classes is an error.
                    if len(unique_classes) == 1 and dets[0] in ALLOWED_CLASSES:
                        reply = dets[0]
                    else:
                        reply = "ERR"
                else:
                    # Original CLASSIFY protocol
                    if len(unique_classes) > 1:
                        reply = f"RESULT {req_id} ERR MULTI"
                    else:
                        cls = dets[0]
                        reply = f"RESULT {req_id} OK {cls}" if cls in ALLOWED_CLASSES \
                            else f"RESULT {req_id} ERR UNKNOWN {cls}"

                stats = f"Auto req {req_id} | det={len(dets)} | {elapsed:.1f} ms"
                self.after(0, lambda: self._show_inference(pil_img, stats))

                # Send reply to STM32
                self.after(0, lambda: self._uart_send(reply))
                self._log(f"Auto inference done – {reply}")

            except Exception as e:
                self._last_class_name = None
                self._last_result_ok  = False
                err_msg = f"RESULT {req_id} ERR FAIL {str(e)[:40]}"
                self.after(0, lambda: self._uart_send(err_msg))
                self._log(f"Auto inference error: {e}")
            finally:
                self._inference_busy = False   # always release the lock

        threading.Thread(target=_infer, daemon=True).start()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _cv2_to_pil(self, frame):
        return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    def _fit_to_canvas(self, img, canvas):
        self.update_idletasks()
        w = max(canvas.winfo_width(),  100)
        h = max(canvas.winfo_height(), 100)
        img = img.copy()
        img.thumbnail((w, h), Image.LANCZOS)
        return img

    def _draw_on_canvas(self, canvas, imgtk):
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        canvas.create_image(w // 2, h // 2, anchor="center", image=imgtk)
        canvas._imgtk = imgtk  # prevent GC

    def _placeholder_canvas(self, canvas, text="No Signal"):
        self.update_idletasks()
        w = max(canvas.winfo_width(),  100)
        h = max(canvas.winfo_height(), 100)
        img = Image.new("RGB", (w, h), "#0a0c12")
        d   = ImageDraw.Draw(img)
        try:
            fnt = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
        except Exception:
            fnt = ImageFont.load_default()
        bbox = d.textbbox((0, 0), text, font=fnt)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        d.text(((w - tw) // 2, (h - th) // 2), text, fill="#3a3f5e", font=fnt)
        return ImageTk.PhotoImage(img)

    # ── UART helpers ──────────────────────────────────────────────────────────

    def _uart_connect(self):
        try:
            self.serctl.connect(self.serial_port_var.get().strip(), 115200)
            self._log("UART connected.")
        except Exception as e:
            self._log(f"UART connect error: {e}")
            messagebox.showerror("UART", str(e))

    def _uart_disconnect(self):
        try:
            self.serctl.disconnect()
            self._log("UART disconnected.")
        except Exception as e:
            self._log(f"UART disconnect error: {e}")

    def _uart_send(self, line: str):
        if not self.serctl.ser:
            self._log("UART not connected.")
            return
        self.serctl.send_line(line)
        self._log(f"TX: {line}")

    def _send_raw_cmd(self):
        cmd = self.raw_cmd_var.get().strip()
        if cmd:
            self._uart_send(cmd)

    def _poll_serial(self):
        try:
            self.serctl.poll()
        finally:
            self.after(30, self._poll_serial)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def on_close(self):
        self.live_running = False
        try:
            self.serctl.disconnect()
        except Exception:
            pass
        if getattr(self, "_cam_proc", None):
            try:
                self._cam_proc.terminate()
            except Exception:
                pass
        self.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = LegoDetectorApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)

    missing = []
    try:
        import PIL
    except ImportError:
        missing.append("Pillow")
    try:
        import cv2
    except ImportError:
        missing.append("opencv-python")
    if not ULTRALYTICS_AVAILABLE:
        missing.append("ultralytics")

    if missing:
        msg  = "Missing Python packages:\n  " + "\n  ".join(missing)
        msg += "\n\nInstall with:\n  pip install " + " ".join(missing)
        messagebox.showwarning("Missing dependencies", msg)

    app.mainloop()
