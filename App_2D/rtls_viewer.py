"""
UWB RTLS 2D Viewer
==================
Receives distance data from a UWB Tag via serial (pyserial),
computes the Tag's 2D position using trilateration, and displays
anchors + tag on a real-time 2D map.

Dependencies: pyserial, matplotlib
    pip install pyserial matplotlib
"""

import re
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
import matplotlib
import math

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches

# ========================= Configuration =========================

DEFAULT_BAUD = 115200
SERIAL_REGEX = re.compile(r"A(\d+):\s*([\d.]+)\s*cm")
RSSI_REGEX = re.compile(r"A(\d+):\s*(-?[\d.]+)\s*dBm")
UPDATE_INTERVAL_MS = 100  # GUI refresh rate

# Default anchor positions (cm)
DEFAULT_ANCHORS = {
    1: (264.0, 20.5),
    2: (0.0, 227.0),
    3: (216.0, 585.0),
}

# ========================= Colors (Dark Theme) =========================

BG_DARK = "#1a1a2e"
BG_PANEL = "#16213e"
BG_INPUT = "#0f3460"
FG_TEXT = "#e0e0e0"
FG_ACCENT = "#00d4ff"
FG_WARN = "#ff6b6b"
FG_SUCCESS = "#51cf66"
FG_ANCHOR = "#4dabf7"
FG_TAG = "#ff6b6b"
FG_GRID = "#2a2a4a"
FG_CIRCLE = "#3a3a6a"


# ========================= Trilateration =========================

def trilaterate(x1, y1, d1, x2, y2, d2, x3, y3, d3):
    """
    Compute tag position from three anchor positions and their distances.
    Uses the closed-form equations (5)-(12) from the reference images.

    Returns (x_tag, y_tag, D1T, D2T, D3T) or None if the system is degenerate.
    """

    # Chuyen doi he khoang cach sang mat phang 2D theo do cao cua Anchor va Tag
    High_Tag = 98
    High_Anchor = 200
    dh = High_Anchor - High_Tag
    D1T = math.sqrt(abs(d1**2 - dh**2))
    D2T = math.sqrt(abs(d2**2 - dh**2))
    D3T = math.sqrt(abs(d3**2 - dh**2))

    # Tinh vi tri tag trong mat phang 2D
    A = -2 * x1 + 2 * x2
    B = -2 * y1 + 2 * y2
    C = D1T**2 - D2T**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = -2 * x2 + 2 * x3
    E = -2 * y2 + 2 * y3
    F = D2T**2 - D3T**2 - x2**2 + x3**2 - y2**2 + y3**2

    denom_x = E * A - B * D
    denom_y = B * D - A * E

    if abs(denom_x) < 1e-9 or abs(denom_y) < 1e-9:
        return None  # Degenerate (collinear anchors or identical positions)

    x_tag = (C * E - F * B) / denom_x
    y_tag = (C * D - A * F) / denom_y

    return (x_tag, y_tag, D1T, D2T, D3T)


# ========================= Serial Reader =========================

class SerialReader:
    """Background thread that reads serial data and updates distances."""

    def __init__(self):
        self.port = None
        self.baud = DEFAULT_BAUD
        self.serial_conn = None
        self.running = False
        self.thread = None
        self.distances = {1: None, 2: None, 3: None}
        self.rssi = {1: None, 2: None, 3: None}
        self.lock = threading.Lock()
        self.raw_line = ""

    def connect(self, port, baud):
        self.port = port
        self.baud = baud
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except serial.SerialException as e:
            return str(e)

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None

    def _read_loop(self):
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                line = self.serial_conn.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                with self.lock:
                    self.raw_line = line
                # Parse: "Distances - A1: 123.45 cm | A2: 67.89 cm | A3: 101.23 cm"
                dist_matches = SERIAL_REGEX.findall(line)
                if dist_matches:
                    with self.lock:
                        for anchor_id_str, dist_str in dist_matches:
                            aid = int(anchor_id_str)
                            if aid in self.distances:
                                try:
                                    self.distances[aid] = float(dist_str)
                                except ValueError:
                                    pass
                # Parse: "RSSI - A1: -85.23 dBm | A2: -79.45 dBm | A3: -82.10 dBm"
                rssi_matches = RSSI_REGEX.findall(line)
                if rssi_matches:
                    with self.lock:
                        for anchor_id_str, rssi_str in rssi_matches:
                            aid = int(anchor_id_str)
                            if aid in self.rssi:
                                try:
                                    self.rssi[aid] = float(rssi_str)
                                except ValueError:
                                    pass
            except Exception:
                if self.running:
                    continue

    def get_distances(self):
        with self.lock:
            return dict(self.distances)

    def get_rssi(self):
        with self.lock:
            return dict(self.rssi)

    def get_raw_line(self):
        with self.lock:
            return self.raw_line

    @property
    def is_connected(self):
        return self.serial_conn is not None and self.serial_conn.is_open


# ========================= Main Application =========================

class RTLSViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UWB RTLS 2D Viewer")
        self.root.configure(bg=BG_DARK)
        self.root.state("zoomed")  # Start maximized on Windows
        self.root.minsize(1100, 700)

        self.serial_reader = SerialReader()
        self.anchors = dict(DEFAULT_ANCHORS)
        self.tag_pos = None
        self.tag_history = []  # Trail of previous tag positions
        self.max_trail = 50

        self._build_styles()
        self._build_ui()
        self._update_loop()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # -------------------- Styles --------------------

    def _build_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("Dark.TFrame", background=BG_DARK)
        style.configure("Panel.TFrame", background=BG_PANEL)
        style.configure(
            "Dark.TLabel",
            background=BG_PANEL,
            foreground=FG_TEXT,
            font=("Segoe UI", 10),
        )
        style.configure(
            "Header.TLabel",
            background=BG_PANEL,
            foreground=FG_ACCENT,
            font=("Segoe UI", 12, "bold"),
        )
        style.configure(
            "Title.TLabel",
            background=BG_DARK,
            foreground=FG_ACCENT,
            font=("Segoe UI", 14, "bold"),
        )
        style.configure(
            "Dist.TLabel",
            background=BG_PANEL,
            foreground=FG_TEXT,
            font=("Consolas", 13, "bold"),
        )
        style.configure(
            "Tag.TLabel",
            background=BG_PANEL,
            foreground=FG_TAG,
            font=("Consolas", 13, "bold"),
        )
        style.configure(
            "Status.TLabel",
            background=BG_DARK,
            foreground=FG_TEXT,
            font=("Segoe UI", 9),
        )
        style.configure(
            "Accent.TButton",
            background=FG_ACCENT,
            foreground="#000",
            font=("Segoe UI", 10, "bold"),
            borderwidth=0,
            padding=(12, 6),
        )
        style.map(
            "Accent.TButton",
            background=[("active", "#00b8d9"), ("disabled", "#555")],
        )
        style.configure(
            "Danger.TButton",
            background=FG_WARN,
            foreground="#000",
            font=("Segoe UI", 10, "bold"),
            borderwidth=0,
            padding=(12, 6),
        )
        style.map(
            "Danger.TButton",
            background=[("active", "#e05555")],
        )

    # -------------------- UI Layout --------------------

    def _build_ui(self):
        # Top bar
        self._build_top_bar()
        # Main content: left panel + map
        main = ttk.Frame(self.root, style="Dark.TFrame")
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        self._build_left_panel(main)
        self._build_map(main)
        # Status bar
        self._build_status_bar()

    def _build_top_bar(self):
        bar = ttk.Frame(self.root, style="Dark.TFrame")
        bar.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(bar, text="⬡ UWB RTLS 2D Viewer", style="Title.TLabel").pack(
            side=tk.LEFT, padx=(4, 20)
        )

        # COM port
        ttk.Label(bar, text="Port:", style="Title.TLabel").pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            bar, textvariable=self.port_var, width=12, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(0, 4))
        self._refresh_ports()

        refresh_btn = ttk.Button(bar, text="⟳", width=3, command=self._refresh_ports)
        refresh_btn.pack(side=tk.LEFT, padx=(0, 12))

        # Baud rate
        ttk.Label(bar, text="Baud:", style="Title.TLabel").pack(side=tk.LEFT, padx=(0, 4))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        baud_combo = ttk.Combobox(
            bar,
            textvariable=self.baud_var,
            values=["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"],
            width=8,
            state="readonly",
        )
        baud_combo.pack(side=tk.LEFT, padx=(0, 12))

        # Connect / Disconnect
        self.connect_btn = ttk.Button(
            bar, text="Connect", style="Accent.TButton", command=self._toggle_connection
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 8))

        # Connection status indicator
        self.conn_indicator = tk.Canvas(bar, width=16, height=16, bg=BG_DARK, highlightthickness=0)
        self.conn_indicator.pack(side=tk.LEFT, padx=(0, 4))
        self._draw_indicator(False)

    def _build_left_panel(self, parent):
        left = ttk.Frame(parent, style="Panel.TFrame", width=320)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.grid_propagate(False)

        pad = {"padx": 12, "pady": 4}
        pad_section = {"padx": 12, "pady": (16, 4)}

        # ---- Anchor Coordinates ----
        ttk.Label(left, text="📍 Anchor Coordinates (cm)", style="Header.TLabel").pack(
            anchor="w", **pad_section
        )
        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, padx=12, pady=2)

        self.anchor_entries = {}
        for aid, (dx, dy) in DEFAULT_ANCHORS.items():
            frame = ttk.Frame(left, style="Panel.TFrame")
            frame.pack(fill=tk.X, **pad)
            ttk.Label(frame, text=f"A{aid}:", style="Dark.TLabel", width=4).pack(side=tk.LEFT)
            ttk.Label(frame, text="X:", style="Dark.TLabel").pack(side=tk.LEFT)
            ex = tk.Entry(
                frame, width=8, bg=BG_INPUT, fg=FG_TEXT, insertbackground=FG_TEXT,
                font=("Consolas", 11), relief="flat", bd=4
            )
            ex.insert(0, str(dx))
            ex.pack(side=tk.LEFT, padx=(0, 8))
            ttk.Label(frame, text="Y:", style="Dark.TLabel").pack(side=tk.LEFT)
            ey = tk.Entry(
                frame, width=8, bg=BG_INPUT, fg=FG_TEXT, insertbackground=FG_TEXT,
                font=("Consolas", 11), relief="flat", bd=4
            )
            ey.insert(0, str(dy))
            ey.pack(side=tk.LEFT)
            self.anchor_entries[aid] = (ex, ey)

        ttk.Button(left, text="Apply Coordinates", style="Accent.TButton",
                    command=self._apply_anchors).pack(pady=(8, 4), padx=12, fill=tk.X)

        # ---- Measured Distances ----
        ttk.Label(left, text="📏 Measured Distances", style="Header.TLabel").pack(
            anchor="w", **pad_section
        )
        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, padx=12, pady=2)

        self.dist_labels = {}
        for aid in [1, 2, 3]:
            frame = ttk.Frame(left, style="Panel.TFrame")
            frame.pack(fill=tk.X, **pad)
            ttk.Label(frame, text=f"  d{aid} (A{aid}):", style="Dark.TLabel", width=10).pack(
                side=tk.LEFT
            )
            lbl = ttk.Label(frame, text="— cm", style="Dist.TLabel")
            lbl.pack(side=tk.LEFT, padx=(8, 0))
            self.dist_labels[aid] = lbl

        # ---- RSSI ----
        ttk.Label(left, text="📶 RSSI (Signal Strength)", style="Header.TLabel").pack(
            anchor="w", **pad_section
        )
        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, padx=12, pady=2)

        self.rssi_labels = {}
        for aid in [1, 2, 3]:
            frame = ttk.Frame(left, style="Panel.TFrame")
            frame.pack(fill=tk.X, **pad)
            ttk.Label(frame, text=f"  A{aid}:", style="Dark.TLabel", width=10).pack(
                side=tk.LEFT
            )
            lbl = ttk.Label(frame, text="— dBm", style="Dist.TLabel")
            lbl.pack(side=tk.LEFT, padx=(8, 0))
            self.rssi_labels[aid] = lbl

        # ---- Actual Position (D1T, D2T, D3T) ----
        ttk.Label(left, text="📐 Actual Position (2D)", style="Header.TLabel").pack(
            anchor="w", **pad_section
        )
        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, padx=12, pady=2)

        self.d_actual_labels = {}
        for i in [1, 2, 3]:
            frame = ttk.Frame(left, style="Panel.TFrame")
            frame.pack(fill=tk.X, **pad)
            ttk.Label(frame, text=f"  D{i}T:", style="Dark.TLabel", width=10).pack(
                side=tk.LEFT
            )
            lbl = ttk.Label(frame, text="— cm", style="Tag.TLabel")
            lbl.pack(side=tk.LEFT, padx=(8, 0))
            self.d_actual_labels[i] = lbl

        # ---- Serial Monitor ----
        ttk.Label(left, text="📡 Serial Monitor", style="Header.TLabel").pack(
            anchor="w", **pad_section
        )
        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, padx=12, pady=2)
        self.serial_text = tk.Text(
            left, height=6, bg=BG_INPUT, fg=FG_TEXT, font=("Consolas", 9),
            relief="flat", bd=4, wrap=tk.WORD, state=tk.DISABLED
        )
        self.serial_text.pack(fill=tk.X, padx=12, pady=4)

    def _build_map(self, parent):
        map_frame = ttk.Frame(parent, style="Dark.TFrame")
        map_frame.grid(row=0, column=1, sticky="nsew")

        self.fig = Figure(facecolor=BG_DARK)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=map_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._draw_map()

    def _build_status_bar(self):
        bar = ttk.Frame(self.root, style="Dark.TFrame")
        bar.pack(fill=tk.X, padx=8, pady=(0, 4))
        self.status_label = ttk.Label(bar, text="Disconnected", style="Status.TLabel")
        self.status_label.pack(side=tk.LEFT)

    # -------------------- Port Management --------------------

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_names = [p.device for p in ports]
        self.port_combo["values"] = port_names
        if port_names and not self.port_var.get():
            self.port_var.set(port_names[0])

    def _toggle_connection(self):
        if self.serial_reader.is_connected:
            self.serial_reader.disconnect()
            self.connect_btn.configure(text="Connect", style="Accent.TButton")
            self._draw_indicator(False)
            self.status_label.configure(text="Disconnected")
        else:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            if not port:
                messagebox.showwarning("No Port", "Please select a COM port.")
                return
            result = self.serial_reader.connect(port, baud)
            if result is True:
                self.connect_btn.configure(text="Disconnect", style="Danger.TButton")
                self._draw_indicator(True)
                self.status_label.configure(text=f"Connected to {port} @ {baud}")
            else:
                messagebox.showerror("Connection Error", f"Could not open {port}:\n{result}")

    def _draw_indicator(self, connected):
        self.conn_indicator.delete("all")
        color = FG_SUCCESS if connected else FG_WARN
        self.conn_indicator.create_oval(2, 2, 14, 14, fill=color, outline="")

    # -------------------- Anchor Coordinate Input --------------------

    def _apply_anchors(self):
        try:
            for aid, (ex, ey) in self.anchor_entries.items():
                x = float(ex.get())
                y = float(ey.get())
                self.anchors[aid] = (x, y)
            # Clear trail on anchor change
            self.tag_history.clear()
            self._draw_map()
        except ValueError:
            messagebox.showwarning("Invalid Input", "Please enter valid numeric coordinates.")

    # -------------------- 2D Map Drawing --------------------

    def _draw_map(self):
        ax = self.ax
        ax.clear()

        ax.set_facecolor(BG_DARK)
        ax.set_xlabel("X (cm)", color=FG_TEXT, fontsize=11)
        ax.set_ylabel("Y (cm)", color=FG_TEXT, fontsize=11)
        ax.set_title("UWB Real-Time Location", color=FG_ACCENT, fontsize=14, fontweight="bold", pad=12)
        ax.tick_params(colors=FG_TEXT, which="both")
        for spine in ax.spines.values():
            spine.set_color(FG_GRID)
        ax.grid(True, color=FG_GRID, linewidth=0.5, alpha=0.6)
        ax.set_aspect("equal", adjustable="box")

        # Draw distance circles
        distances = self.serial_reader.get_distances()
        for aid, (ax_pos, ay_pos) in self.anchors.items():
            d = distances.get(aid)
            if d is not None and d > 0:
                circle = patches.Circle(
                    (ax_pos, ay_pos), d,
                    fill=False, edgecolor=FG_CIRCLE, linewidth=1.2,
                    linestyle="--", alpha=0.5
                )
                ax.add_patch(circle)

        # Draw tag trail
        if len(self.tag_history) > 1:
            trail_x = [p[0] for p in self.tag_history]
            trail_y = [p[1] for p in self.tag_history]
            # Gradient alpha for trail
            for i in range(len(trail_x) - 1):
                alpha = 0.1 + 0.4 * (i / max(len(trail_x) - 1, 1))
                ax.plot(
                    trail_x[i:i+2], trail_y[i:i+2],
                    color=FG_TAG, alpha=alpha, linewidth=1.5
                )

        # Draw tag position
        if self.tag_pos:
            ax.plot(
                self.tag_pos[0], self.tag_pos[1],
                "o", color=FG_TAG, markersize=12,
                markeredgecolor="#fff", markeredgewidth=1.5, zorder=10
            )
            ax.annotate(
                f"Tag\n({self.tag_pos[0]:.1f}, {self.tag_pos[1]:.1f})",
                (self.tag_pos[0], self.tag_pos[1]),
                textcoords="offset points", xytext=(14, 14),
                color=FG_TAG, fontsize=9, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor=BG_PANEL, edgecolor=FG_TAG, alpha=0.85)
            )

        # Draw anchors (drawn last so they appear on top)
        for aid, (ax_pos, ay_pos) in self.anchors.items():
            ax.plot(
                ax_pos, ay_pos,
                "D", color=FG_ANCHOR, markersize=10,
                markeredgecolor="#fff", markeredgewidth=1.5, zorder=11
            )
            dist_text = ""
            d = distances.get(aid)
            if d is not None:
                dist_text = f"\nd={d:.1f}"
            ax.annotate(
                f"A{aid}\n({ax_pos:.0f},{ay_pos:.0f}){dist_text}",
                (ax_pos, ay_pos),
                textcoords="offset points", xytext=(12, -18),
                color=FG_ANCHOR, fontsize=9, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor=BG_PANEL, edgecolor=FG_ANCHOR, alpha=0.85)
            )

        self.fig.tight_layout()
        self.canvas.draw_idle()

    # -------------------- Update Loop --------------------

    def _update_loop(self):
        distances = self.serial_reader.get_distances()

        # Update distance labels
        for aid in [1, 2, 3]:
            d = distances.get(aid)
            if d is not None:
                self.dist_labels[aid].configure(text=f"{d:.2f} cm")
            else:
                self.dist_labels[aid].configure(text="— cm")

        # Update RSSI labels
        rssi_vals = self.serial_reader.get_rssi()
        for aid in [1, 2, 3]:
            r = rssi_vals.get(aid)
            if r is not None:
                self.rssi_labels[aid].configure(text=f"{r:.2f} dBm")
            else:
                self.rssi_labels[aid].configure(text="— dBm")

        # Compute trilateration
        d1, d2, d3 = distances.get(1), distances.get(2), distances.get(3)
        if d1 is not None and d2 is not None and d3 is not None:
            x1, y1 = self.anchors[1]
            x2, y2 = self.anchors[2]
            x3, y3 = self.anchors[3]
            result = trilaterate(x1, y1, d1, x2, y2, d2, x3, y3, d3)
            if result:
                x_tag, y_tag, D1T, D2T, D3T = result
                self.tag_pos = (x_tag, y_tag)
                self.tag_history.append((x_tag, y_tag))
                if len(self.tag_history) > self.max_trail:
                    self.tag_history.pop(0)
                # Update Actual Position labels
                self.d_actual_labels[1].configure(text=f"{D1T:.2f} cm")
                self.d_actual_labels[2].configure(text=f"{D2T:.2f} cm")
                self.d_actual_labels[3].configure(text=f"{D3T:.2f} cm")
            else:
                for i in [1, 2, 3]:
                    self.d_actual_labels[i].configure(text="ERROR")
        else:
            for i in [1, 2, 3]:
                self.d_actual_labels[i].configure(text="— cm")

        # Update serial monitor
        raw = self.serial_reader.get_raw_line()
        if raw:
            self.serial_text.configure(state=tk.NORMAL)
            self.serial_text.insert(tk.END, raw + "\n")
            self.serial_text.see(tk.END)
            # Keep buffer limited
            lines = int(self.serial_text.index("end-1c").split(".")[0])
            if lines > 200:
                self.serial_text.delete("1.0", "100.0")
            self.serial_text.configure(state=tk.DISABLED)

        # Redraw map
        self._draw_map()

        # Schedule next update
        self.root.after(UPDATE_INTERVAL_MS, self._update_loop)

    # -------------------- Cleanup --------------------

    def _on_close(self):
        self.serial_reader.disconnect()
        self.root.destroy()


# ========================= Entry Point =========================

if __name__ == "__main__":
    root = tk.Tk()
    app = RTLSViewerApp(root)
    root.mainloop()
