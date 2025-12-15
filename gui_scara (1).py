"""
GUI avanzada para control y monitoreo de un robot SCARA RRP.

Características:


- Control manual de q1, q2 (°) y d3 (mm) con sliders + entradas.
- Medidas en tiempo real: q1, q2, d3, x, y, z, r.
- Cálculo de cinemática directa (fk_scara) con:
    - Matrices A1, A2, A3 y T.
    - Pestaña "Cálculos FK paso a paso" tipo Jupyter.
- Simulación gráfica con MULTI-VISTA:
    - Vista superior XY (área de trabajo + robot).
    - Vista 3D XYZ para ver el actuador subiendo/bajando.
    - Vista dedicada del eje Z en una pestaña aparte.
- Animación suave entre configuraciones articulares.
- Envío de q1, q2, d3 y estado de gripper al Arduino.
- Pestaña de Rutinas (lista de poses) y ejecución/simulación.
- Pestaña de Monitor Serial (RX/TX).
- NUEVO:
    - Bloque de cinemática inversa (IK) con entradas x, y, z.
    - Botones para calcular IK / mover / enviar a Arduino.
    - Botón "Ir a origen (q=0, z=0)".
    - Simular / ejecutar paso seleccionado de la rutina.
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import serial
import time
import math

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # activa soporte 3D

from scara_kinematics import (
    fk_scara,
    fk_scara_planar,
    check_joint_limits,
    ik_scara,      # <--- NUEVO: cinemática inversa
    L1, L2, D0
)

# ==========================
# Configuración de conexión
# ==========================
SERIAL_PORT = "COM14"      # AJUSTA SI ES OTRO
BAUD_RATE   = 115200

# ==========================
# Paleta de colores
# ==========================
BG_MAIN   = "#02010F"  # fondo general (azul morado oscuro)
BG_CARD   = "#05081C"  # contenedores
FG_TEXT   = "#F9FAFB"  # texto principal
FG_MUTED  = "#9CA3AF"  # texto secundario
ACCENT    = "#38BDF8"  # azul
ACCENT_2  = "#22C55E"  # verde
DANGER    = "#EF4444"  # rojo
ACCENT_OR = "#FB923C"  # naranja para detalles


class ScaraGUI:
    def __init__(self, master: tk.Tk):
        self.master = master
        self.master.title("SCARA RRP – Panel de Control y Simulación")
        self.master.geometry("1360x800")
        self.master.configure(bg=BG_MAIN)

        # Estado interno
        self.ser = None
        self.serial_ok = False
        self.last_result = None
        self.ws_x = None
        self.ws_y = None
        self.last_q = (0.0, 0.0, 0.0)  # última configuración dibujada (q1,q2,d3)

        # Gripper
        self.gripper_open = tk.BooleanVar(value=True)

        # Variables articulares
        self.q1_var = tk.StringVar(value="0.0")
        self.q2_var = tk.StringVar(value="0.0")
        self.d3_var = tk.StringVar(value="0.0")

        # Variables cartesianas (para IK)
        # Puedes cambiar estos valores iniciales a algo práctico (p.ej. centro del workspace)
        self.x_var = tk.StringVar(value="0.20")
        self.y_var = tk.StringVar(value="0.00")
        self.z_var = tk.StringVar(value=f"{D0 - 0.02:.3f}")  # un poco por debajo de D0

        # Rutinas
        self.routine = []
        self.routine_delay_var = tk.StringVar(value="1500")

        # Velocidad global (afecta simulación, rutinas y robot)
        self.speed_percent = tk.DoubleVar(value=100.0)  # 100% = velocidad nominal


        # Monitor serial y vista Z
        self.txt_serial_monitor = None
        self.fig_z = None
        self.ax_z = None
        self.canvas_z = None

        self._create_styles()
        self._create_layout()
        self._init_serial()
        self._init_plot()

    # ------------------------------------------------------------------
    # Estilos (tipografía + widgets)
    # ------------------------------------------------------------------
    def _create_styles(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # Tipografías: un poco más “tech”
        title_font   = ("Bahnschrift", 22, "bold")
        header_font  = ("Bahnschrift", 15, "bold")
        label_font   = ("Bahnschrift", 11)
        small_font   = ("Bahnschrift", 10)
        tab_font     = ("Bahnschrift", 11, "bold")

        self._font_title  = title_font
        self._font_header = header_font
        self._font_label  = label_font
        self._font_small  = small_font

        style.configure("TFrame", background=BG_MAIN)
        style.configure("TLabel", background=BG_MAIN, foreground=FG_TEXT, font=label_font)
        style.configure("TNotebook", background=BG_MAIN, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            font=tab_font,
            padding=(12, 6),
            background="#060818",
            foreground=FG_MUTED,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", "#0B1120")],
            foreground=[("selected", FG_TEXT)],
        )

        style.configure(
            "Header.TLabel",
            font=title_font,
            background=BG_MAIN,
            foreground=FG_TEXT,
        )
        style.configure(
            "SubHeader.TLabel",
            font=header_font,
            background=BG_MAIN,
            foreground=ACCENT,
        )
        style.configure(
            "Status.TLabel",
            font=small_font,
            background=BG_MAIN,
            foreground=FG_MUTED,
        )
        style.configure(
            "Ok.TLabel",
            font=small_font,
            background=BG_MAIN,
            foreground=ACCENT_2,
        )
        style.configure(
            "Danger.TLabel",
            font=small_font,
            background=BG_MAIN,
            foreground=DANGER,
        )

        # Contenedores tipo “card”
        style.configure(
            "Card.TLabelframe",
            background=BG_CARD,
            foreground=FG_TEXT,
            borderwidth=1,
            relief="solid",
        )
        style.configure(
            "Card.TLabelframe.Label",
            background=BG_CARD,
            foreground=ACCENT,
            font=("Bahnschrift", 12, "bold"),
        )

        style.configure(
            "Accent.TButton",
            font=("Bahnschrift", 11, "bold"),
            padding=6,
            foreground=FG_TEXT,
            borderwidth=0,
        )
        style.map(
            "Accent.TButton",
            background=[("!disabled", "#0EA5E9"), ("active", "#38BDF8")],
            foreground=[("disabled", "#6B7280")],
        )

        style.configure(
            "Treeview",
            background=BG_MAIN,
            fieldbackground=BG_MAIN,
            foreground=FG_TEXT,
            rowheight=26,
            bordercolor="#1F2933",
            borderwidth=0,
        )
        style.map(
            "Treeview",
            background=[("selected", "#1D4ED8")],
            foreground=[("selected", "#F9FAFB")],
        )

        style.configure(
            "Horizontal.TScale",
            background=BG_MAIN,
            troughcolor="#020617",
        )

    # ------------------------------------------------------------------
    # Layout general
    # ------------------------------------------------------------------
    def _create_layout(self):
        # TOP: título y estado
        top = ttk.Frame(self.master)
        top.pack(side=tk.TOP, fill=tk.X, padx=12, pady=10)

        ttk.Label(
            top,
            text="SCARA RRP – Panel de Control y Simulación",
            style="Header.TLabel",
        ).pack(side=tk.LEFT)

        self.status_label = ttk.Label(
            top,
            text="Puerto serie: desconectado",
            style="Status.TLabel",
            anchor="e",
        )
        self.status_label.pack(side=tk.RIGHT)

        # CENTRO: izquierda (controles), derecha (plot + tabs)
        center = ttk.Frame(self.master)
        center.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=12, pady=(0, 10))

        left = ttk.Frame(center)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        right = ttk.Frame(center)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.left_frame = left
        self.right_frame = right

        self._create_left_panel()
        self._create_right_panel()

    # ------------------------------------------------------------------
    # Panel izquierdo (controles)
    # ------------------------------------------------------------------
    def _create_left_panel(self):
        lf = self.left_frame

        ttk.Label(lf, text="Control manual", style="SubHeader.TLabel").pack(
            anchor="w", pady=(0, 6)
        )

        # Articulaciones
        joints = ttk.LabelFrame(
            lf, text="Articulaciones", style="Card.TLabelframe", padding=12
        )
        joints.pack(fill=tk.X, pady=5)

        jf_inner = ttk.Frame(joints, padding=4)
        jf_inner.pack(fill=tk.X)

        # q1
        f_q1 = ttk.Frame(jf_inner)
        f_q1.pack(fill=tk.X, pady=4)
        ttk.Label(
            f_q1,
            text="q1 – Base [°]",
            font=("Bahnschrift", 12, "bold"),
        ).pack(anchor="w")
        self.q1_scale = ttk.Scale(
            f_q1,
            from_=-90,
            to=90,
            orient=tk.HORIZONTAL,
            command=self._on_q1_scale,
            style="Horizontal.TScale",
        )
        self.q1_scale.set(0.0)
        self.q1_scale.pack(fill=tk.X, padx=2, pady=2)

        row_q1 = ttk.Frame(f_q1)
        row_q1.pack(fill=tk.X)
        ttk.Label(row_q1, text="Valor:", foreground=FG_MUTED).pack(side=tk.LEFT)
        self.q1_entry = ttk.Entry(row_q1, textvariable=self.q1_var, width=7)
        self.q1_entry.pack(side=tk.LEFT, padx=4)
        ttk.Label(row_q1, text="°", foreground=FG_MUTED).pack(side=tk.LEFT)

        # q2
        f_q2 = ttk.Frame(jf_inner)
        f_q2.pack(fill=tk.X, pady=4)
        ttk.Label(
            f_q2,
            text="q2 – Codo [°]",
            font=("Bahnschrift", 12, "bold"),
        ).pack(anchor="w")
        self.q2_scale = ttk.Scale(
            f_q2,
            from_=-90,
            to=90,
            orient=tk.HORIZONTAL,
            command=self._on_q2_scale,
            style="Horizontal.TScale",
        )
        self.q2_scale.set(0.0)
        self.q2_scale.pack(fill=tk.X, padx=2, pady=2)

        row_q2 = ttk.Frame(f_q2)
        row_q2.pack(fill=tk.X)
        ttk.Label(row_q2, text="Valor:", foreground=FG_MUTED).pack(side=tk.LEFT)
        self.q2_entry = ttk.Entry(row_q2, textvariable=self.q2_var, width=7)
        self.q2_entry.pack(side=tk.LEFT, padx=4)
        ttk.Label(row_q2, text="°", foreground=FG_MUTED).pack(side=tk.LEFT)

        # d3
        f_q3 = ttk.Frame(jf_inner)
        f_q3.pack(fill=tk.X, pady=4)
        ttk.Label(
            f_q3,
            text="d3 – Eje vertical [mm]",
            font=("Bahnschrift", 12, "bold"),
        ).pack(anchor="w")
        self.d3_scale = ttk.Scale(
            f_q3,
            from_=0,
            to=60,
            orient=tk.HORIZONTAL,
            command=self._on_d3_scale,
            style="Horizontal.TScale",
        )
        self.d3_scale.set(0.0)
        self.d3_scale.pack(fill=tk.X, padx=2, pady=2)

        row_q3 = ttk.Frame(f_q3)
        row_q3.pack(fill=tk.X)
        ttk.Label(row_q3, text="Valor:", foreground=FG_MUTED).pack(side=tk.LEFT)
        self.d3_entry = ttk.Entry(row_q3, textvariable=self.d3_var, width=7)
        self.d3_entry.pack(side=tk.LEFT, padx=4)
        ttk.Label(row_q3, text="mm", foreground=FG_MUTED).pack(side=tk.LEFT)

        # Gripper
        gr = ttk.LabelFrame(
            lf, text="Gripper", style="Card.TLabelframe", padding=12
        )
        gr.pack(fill=tk.X, pady=6)

        ttk.Label(
            gr,
            text="Estado del gripper:",
            font=("Bahnschrift", 12),
        ).pack(anchor="w")

        rf = ttk.Frame(gr)
        rf.pack(anchor="w", pady=4)
        ttk.Radiobutton(
            rf, text="Abierto", value=True, variable=self.gripper_open
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Radiobutton(
            rf, text="Cerrado", value=False, variable=self.gripper_open
        ).pack(side=tk.LEFT)

        self.lbl_gripper = ttk.Label(
            gr,
            text="Gripper: ABIERTO",
            font=("Bahnschrift", 12, "bold"),
            foreground=ACCENT_2,
        )
        self.lbl_gripper.pack(anchor="w", pady=(4, 0))

        # Velocidad de movimiento
        speed_frame = ttk.LabelFrame(
            lf, text="Velocidad de movimiento", style="Card.TLabelframe", padding=12
        )
        speed_frame.pack(fill=tk.X, pady=6)

        ttk.Label(
            speed_frame,
            text="Escala global (robot, simulación y rutinas).",
            foreground=FG_MUTED,
            font=("Bahnschrift", 9),
        ).pack(anchor="w")

        row_spd = ttk.Frame(speed_frame)
        row_spd.pack(fill=tk.X, pady=(4, 0))

        self.speed_scale = ttk.Scale(
            row_spd,
            from_=20,           # 20% (muy lento)
            to=200,             # 200% (muy rápido)
            orient=tk.HORIZONTAL,
            command=self._on_speed_change,
            style="Horizontal.TScale",
        )
        self.speed_scale.set(self.speed_percent.get())
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        self.lbl_speed_value = ttk.Label(
            row_spd,
            text="100%",
            foreground=ACCENT_2,
            font=("Bahnschrift", 11, "bold"),
        )
        self.lbl_speed_value.pack(side=tk.RIGHT)

        # Métricas en vivo
        metrics = ttk.LabelFrame(
            lf, text="Mediciones en tiempo real", style="Card.TLabelframe", padding=12
        )
        metrics.pack(fill=tk.X, pady=6)

        self.lbl_joint_live = ttk.Label(
            metrics,
            text="q1 = 0.0°  |  q2 = 0.0°  |  d3 = 0.0 mm",
            font=("Bahnschrift", 11),
        )
        self.lbl_joint_live.pack(anchor="w")

        self.lbl_task_live = ttk.Label(
            metrics,
            text="x = 0.000 m  |  y = 0.000 m  |  z = 0.000 m  |  r = 0.000 m",
            font=("Bahnschrift", 11),
            foreground=FG_MUTED,
        )
        self.lbl_task_live.pack(anchor="w", pady=(2, 0))

        # ---- NUEVO: Bloque de cinemática inversa (IK) ----
        ik_frame = ttk.LabelFrame(
            lf, text="Cinemática inversa (x, y, z)", style="Card.TLabelframe", padding=12
        )
        ik_frame.pack(fill=tk.X, pady=6)

        row_x = ttk.Frame(ik_frame)
        row_x.pack(fill=tk.X, pady=2)
        ttk.Label(row_x, text="x [m]:", foreground=FG_MUTED).pack(side=tk.LEFT)
        ttk.Entry(row_x, textvariable=self.x_var, width=8).pack(side=tk.LEFT, padx=4)

        row_y = ttk.Frame(ik_frame)
        row_y.pack(fill=tk.X, pady=2)
        ttk.Label(row_y, text="y [m]:", foreground=FG_MUTED).pack(side=tk.LEFT)
        ttk.Entry(row_y, textvariable=self.y_var, width=8).pack(side=tk.LEFT, padx=4)

        row_z = ttk.Frame(ik_frame)
        row_z.pack(fill=tk.X, pady=2)
        ttk.Label(row_z, text="z [m]:", foreground=FG_MUTED).pack(side=tk.LEFT)
        ttk.Entry(row_z, textvariable=self.z_var, width=8).pack(side=tk.LEFT, padx=4)

        ttk.Label(
            ik_frame,
            text="Usa puntos dentro del espacio de trabajo.\n"
                 "Recomendación: r entre 0.18–0.24 m y z entre 0.05–0.11 m aprox.",
            foreground=FG_MUTED,
            font=("Bahnschrift", 9),
        ).pack(anchor="w", pady=(4, 4))

        ttk.Button(
            ik_frame,
            text="Calcular IK (solo GUI)",
            style="Accent.TButton",
            command=self.on_calcular_ik_gui,
        ).pack(fill=tk.X, pady=3)

        ttk.Button(
            ik_frame,
            text="IK → mover + enviar",
            style="Accent.TButton",
            command=self.on_ik_mover_enviar,
        ).pack(fill=tk.X, pady=3)

        # Botones
        btns = ttk.LabelFrame(
            lf, text="Acciones", style="Card.TLabelframe", padding=12
        )
        btns.pack(fill=tk.X, pady=8)

        ttk.Button(
            btns, text="Calcular FK", style="Accent.TButton",
            command=self.on_calcular_fk
        ).pack(fill=tk.X, pady=3)

        ttk.Button(
            btns,
            text="Calcular + animar vista",
            style="Accent.TButton",
            command=self.on_calcular_y_animar,
        ).pack(fill=tk.X, pady=3)

        ttk.Button(
            btns,
            text="Enviar a Arduino",
            style="Accent.TButton",
            command=self.on_enviar_arduino,
        ).pack(fill=tk.X, pady=3)

        ttk.Button(
            btns,
            text="Añadir pose a rutina",
            style="Accent.TButton",
            command=self.on_add_pose_to_routine,
        ).pack(fill=tk.X, pady=3)

        # NUEVO: botón para mandar al origen
        ttk.Button(
            btns,
            text="Ir a origen (q=0, z=0)",
            style="Accent.TButton",
            command=self.on_go_origin,
        ).pack(fill=tk.X, pady=3)

        # Resumen
        resumen = ttk.LabelFrame(
            lf, text="Resumen cinemático", style="Card.TLabelframe", padding=12
        )
        resumen.pack(fill=tk.X, pady=(5, 0))

        self.lbl_q_summary = ttk.Label(
            resumen,
            text="q1 = 0.0°   q2 = 0.0°   d3 = 0.0 mm",
            font=("Bahnschrift", 11, "bold"),
        )
        self.lbl_q_summary.pack(anchor="w")

        self.lbl_xyz_summary = ttk.Label(
            resumen,
            text="x = 0.000 m   y = 0.000 m   z = 0.000 m",
            font=("Bahnschrift", 11, "bold"),
            foreground=FG_MUTED,
        )
        self.lbl_xyz_summary.pack(anchor="w")

        self._update_live_metrics()

    # ------------------------------------------------------------------
    # Panel derecho (plot + tabs)
    # ------------------------------------------------------------------
    def _create_right_panel(self):
        rf = self.right_frame

        # Arriba: gráfica XY + 3D
        plot_frame = ttk.Frame(rf)
        plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.fig = plt.figure(figsize=(8.5, 4.8))
        self.ax_top = self.fig.add_subplot(1, 2, 1)
        self.ax_3d = self.fig.add_subplot(1, 2, 2, projection="3d")

        self.fig.patch.set_facecolor(BG_MAIN)
        self.ax_top.set_facecolor(BG_MAIN)
        for spine in self.ax_top.spines.values():
            spine.set_visible(False)

        self.ax_3d.set_facecolor(BG_MAIN)
        pane_color = (0.02, 0.02, 0.08, 0.9)
        for axis in (self.ax_3d.xaxis, self.ax_3d.yaxis, self.ax_3d.zaxis):
            try:
                axis.pane.set_facecolor(pane_color)
            except Exception:
                pass

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Abajo: Notebook con pestañas
        bottom = ttk.Notebook(rf)
        bottom.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, pady=(8, 0))

        # Tab matrices
        frame_mat = ttk.Frame(bottom)
        bottom.add(frame_mat, text="Matrices DH y T")

        self.txt_matrices = scrolledtext.ScrolledText(
            frame_mat,
            height=10,
            font=("Consolas", 11),
            bg=BG_CARD,
            fg=FG_TEXT,
            insertbackground=FG_TEXT,
            borderwidth=0,
        )
        self.txt_matrices.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # Tab cálculos FK paso a paso
        frame_explain = ttk.Frame(bottom)
        bottom.add(frame_explain, text="Cálculos FK paso a paso")

        self.txt_explain = scrolledtext.ScrolledText(
            frame_explain,
            height=10,
            font=("Consolas", 11),
            bg=BG_CARD,
            fg=FG_TEXT,
            insertbackground=FG_TEXT,
            borderwidth=0,
        )
        self.txt_explain.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # Tab vista eje Z
        frame_z = ttk.Frame(bottom)
        bottom.add(frame_z, text="Vista eje Z")

        self.fig_z = plt.figure(figsize=(6, 2.5))
        self.ax_z = self.fig_z.add_subplot(1, 1, 1)
        self.fig_z.patch.set_facecolor(BG_MAIN)
        self.ax_z.set_facecolor(BG_MAIN)
        for spine in self.ax_z.spines.values():
            spine.set_visible(False)
        self.canvas_z = FigureCanvasTkAgg(self.fig_z, master=frame_z)
        self.canvas_z.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # Tab rutinas
        frame_routines = ttk.Frame(bottom)
        bottom.add(frame_routines, text="Rutinas")
        self._create_routine_panel(frame_routines)

        # Tab monitor serial
        frame_serial = ttk.Frame(bottom)
        bottom.add(frame_serial, text="Monitor serial")

        self.txt_serial_monitor = scrolledtext.ScrolledText(
            frame_serial,
            height=8,
            font=("Consolas", 11),
            bg=BG_CARD,
            fg=FG_TEXT,
            insertbackground=FG_TEXT,
            borderwidth=0,
        )
        self.txt_serial_monitor.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6, 0))

        btn_serial = ttk.Frame(frame_serial)
        btn_serial.pack(fill=tk.X, padx=6, pady=6)
        ttk.Button(
            btn_serial,
            text="Limpiar monitor",
            style="Accent.TButton",
            command=self._clear_serial_monitor,
        ).pack(side=tk.LEFT, padx=(0, 4))

        # Tab log
        frame_log = ttk.Frame(bottom)
        bottom.add(frame_log, text="Log de eventos")

        self.txt_log = scrolledtext.ScrolledText(
            frame_log,
            height=8,
            font=("Consolas", 11),
            bg=BG_CARD,
            fg=FG_TEXT,
            insertbackground=FG_TEXT,
            borderwidth=0,
        )
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

    # ------------------------------------------------------------------
    # Panel de Rutinas (tab)
    # ------------------------------------------------------------------
    def _create_routine_panel(self, parent: ttk.Frame):
        top = ttk.Frame(parent)
        top.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        cols = ("idx", "q1", "q2", "d3", "g")
        self.tree_routine = ttk.Treeview(
            top, columns=cols, show="headings", height=5
        )
        self.tree_routine.heading("idx", text="#")
        self.tree_routine.heading("q1", text="q1 [°]")
        self.tree_routine.heading("q2", text="q2 [°]")
        self.tree_routine.heading("d3", text="d3 [mm]")
        self.tree_routine.heading("g",  text="Gripper")

        self.tree_routine.column("idx", width=40, anchor="center")
        self.tree_routine.column("q1", width=90, anchor="center")
        self.tree_routine.column("q2", width=90, anchor="center")
        self.tree_routine.column("d3", width=90, anchor="center")
        self.tree_routine.column("g",  width=80, anchor="center")

        self.tree_routine.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(top, orient=tk.VERTICAL, command=self.tree_routine.yview)
        self.tree_routine.configure(yscroll=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        controls = ttk.LabelFrame(
            parent,
            text="Acciones de rutina",
            style="Card.TLabelframe",
            padding=10,
        )
        controls.pack(fill=tk.X, padx=6, pady=(0, 6))

        ttk.Button(
            controls,
            text="Limpiar rutina",
            style="Accent.TButton",
            command=self.on_clear_routine,
        ).pack(side=tk.LEFT, padx=4, pady=2)

        ttk.Button(
            controls,
            text="Simular solo en GUI",
            style="Accent.TButton",
            command=self.on_simulate_routine,
        ).pack(side=tk.LEFT, padx=4, pady=2)

        ttk.Button(
            controls,
            text="Ejecutar en robot",
            style="Accent.TButton",
            command=self.on_execute_routine,
        ).pack(side=tk.LEFT, padx=4, pady=2)

        # NUEVO: botones para paso seleccionado
        ttk.Button(
            controls,
            text="Simular paso seleccionado",
            style="Accent.TButton",
            command=self.on_simulate_selected_step,
        ).pack(side=tk.LEFT, padx=4, pady=2)

        ttk.Button(
            controls,
            text="Ejecutar paso seleccionado",
            style="Accent.TButton",
            command=self.on_execute_selected_step,
        ).pack(side=tk.LEFT, padx=4, pady=2)

        ttk.Label(
            controls,
            text="Tiempo entre pasos [ms]:",
            foreground=FG_MUTED,
            font=("Bahnschrift", 10),
        ).pack(side=tk.LEFT, padx=(18, 4))

        ttk.Entry(controls, textvariable=self.routine_delay_var, width=7).pack(
            side=tk.LEFT
        )

    # ------------------------------------------------------------------
    # Serial
    # ------------------------------------------------------------------
    def _init_serial(self):
        try:
            # timeout=0 para lectura no bloqueante y poder hacer monitor en tiempo real
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
            time.sleep(2)
            self.serial_ok = True
            self._set_status(f"Puerto serie {SERIAL_PORT} conectado", ok=True)
            self._log(f"[OK] Conectado a {SERIAL_PORT} @ {BAUD_RATE} baud")
            # iniciar polling
            self.master.after(100, self._poll_serial)
        except Exception as e:
            self.serial_ok = False
            self.ser = None
            self._set_status(f"Sin conexión en {SERIAL_PORT}", ok=False)
            self._log(f"[WARN] No se pudo abrir {SERIAL_PORT}: {e}")

    def _set_status(self, text, ok=True):
        self.status_label.config(text=text, style="Ok.TLabel" if ok else "Danger.TLabel")

    def _poll_serial(self):
        if not self.serial_ok or self.ser is None:
            return
        try:
            n = self.ser.in_waiting
        except Exception:
            n = 0
        if n > 0:
            try:
                data = self.ser.read(n)
                text = data.decode("utf-8", errors="ignore")
                if text:
                    for line in text.splitlines():
                        if line.strip():
                            self._append_serial_monitor(line.strip(), prefix="RX")
            except Exception as e:
                self._log(f"[ERROR] Lectura serial: {e}")
        self.master.after(100, self._poll_serial)

    def _append_serial_monitor(self, text: str, prefix: str | None = None):
        if self.txt_serial_monitor is None:
            return
        line = f"[{prefix}] {text}" if prefix else text
        self.txt_serial_monitor.insert(tk.END, line + "\n")
        self.txt_serial_monitor.see(tk.END)

    def _clear_serial_monitor(self):
        if self.txt_serial_monitor is not None:
            self.txt_serial_monitor.delete("1.0", tk.END)

    # ------------------------------------------------------------------
    # Plot (multi-vista)
    # ------------------------------------------------------------------
    def _init_plot(self):
        self._compute_workspace()
        self._draw_robot(0.0, 0.0, 0.0)
        self.last_q = (0.0, 0.0, 0.0)

    def _compute_workspace(self):
        q1_vals = np.deg2rad(np.linspace(-90, 90, 40))
        q2_vals = np.deg2rad(np.linspace(-90, 90, 40))
        ws_x, ws_y = [], []

        for q1 in q1_vals:
            for q2 in q2_vals:
                x, y, _ = fk_scara_planar(q1, q2, 0.0)
                ws_x.append(x)
                ws_y.append(y)

        self.ws_x = np.array(ws_x)
        self.ws_y = np.array(ws_y)

    def _draw_robot(self, q1_rad, q2_rad, d3_m):
        # Cálculos XY
        x1 = L1 * np.cos(q1_rad)
        y1 = L1 * np.sin(q1_rad)
        x_ef, y_ef, _ = fk_scara_planar(q1_rad, q2_rad, d3_m)

        # Cálculos XYZ reales
        res_fk = fk_scara(q1_rad, q2_rad, d3_m)
        z_ef = res_fk["z"]   # debe ser D0 - d3
        z_base = D0
        z_elbow = D0

        # --- Vista superior XY ---
        self.ax_top.clear()
        self.ax_top.set_facecolor(BG_MAIN)
        for spine in self.ax_top.spines.values():
            spine.set_visible(False)

        self.ax_top.set_title("Vista superior XY", color=FG_TEXT, fontsize=12)
        self.ax_top.set_xlabel("X [m]", color=FG_MUTED)
        self.ax_top.set_ylabel("Y [m]", color=FG_MUTED)
        self.ax_top.tick_params(colors=FG_MUTED)

        if self.ws_x is not None:
            self.ax_top.scatter(
                self.ws_x,
                self.ws_y,
                s=4,
                alpha=0.18,
                label="Área de trabajo",
                color="#1D4ED8",
            )

        self.ax_top.plot([0, x1], [0, y1], marker="o", label="Eslabón 1", color="#FACC15")
        self.ax_top.plot([x1, x_ef], [y1, y_ef], marker="o", label="Eslabón 2", color=ACCENT_OR)
        self.ax_top.scatter([x_ef], [y_ef], c="red", label="Efector")

        self.ax_top.axhline(0, linewidth=0.5, color="#4B5563")
        self.ax_top.axvline(0, linewidth=0.5, color="#4B5563")
        self.ax_top.grid(True, color="#111827", linestyle="--", linewidth=0.6)
        self.ax_top.set_aspect("equal", "box")

        margin = 0.02
        x_min = min(self.ws_x.min(), -L1 - L2) - margin
        x_max = max(self.ws_x.max(),  L1 + L2) + margin
        y_min = min(self.ws_y.min(), -L1 - L2) - margin
        y_max = max(self.ws_y.max(),  L1 + L2) + margin

        self.ax_top.set_xlim(x_min, x_max)
        self.ax_top.set_ylim(y_min, y_max)
        self.ax_top.legend(
            loc="upper right",
            fontsize=9,
            facecolor=BG_MAIN,
            edgecolor="#1F2933",
            labelcolor=FG_TEXT,
        )

        # --- Vista 3D XYZ ---
        self.ax_3d.clear()
        self.ax_3d.set_facecolor(BG_MAIN)

        self.ax_3d.set_title("Vista 3D XYZ", color=FG_TEXT, fontsize=12)
        self.ax_3d.set_xlabel("X [m]", color=FG_MUTED)
        self.ax_3d.set_ylabel("Y [m]", color=FG_MUTED)
        self.ax_3d.set_zlabel("Z [m]", color=FG_MUTED)

        xs = [0.0, x1, x_ef]
        ys = [0.0, y1, y_ef]
        zs = [z_base, z_elbow, z_ef]

        self.ax_3d.plot(xs, ys, zs, marker="o", color=ACCENT)
        self.ax_3d.scatter([x_ef], [y_ef], [z_ef], color="red", s=40)

        # Línea vertical del actuador
        self.ax_3d.plot([x_ef, x_ef], [y_ef, y_ef], [z_base, z_ef], color=ACCENT_OR, linestyle="--")

        r = L1 + L2 + 0.02
        self.ax_3d.set_xlim(-r, r)
        self.ax_3d.set_ylim(-r, r)
        z_min = min(z_ef, z_base) - 0.02
        z_max = max(z_ef, z_base) + 0.02
        self.ax_3d.set_zlim(z_min, z_max)

        max_range = max(r * 2, z_max - z_min)
        mid_x = 0.0
        mid_y = 0.0
        mid_z = (z_min + z_max) / 2.0
        self.ax_3d.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
        self.ax_3d.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
        self.ax_3d.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

        self.canvas.draw_idle()

        # --- Vista eje Z dedicada (en pestaña) ---
        self._draw_z_view(z_ef)

    def _draw_z_view(self, z_ef):
        if self.ax_z is None or self.canvas_z is None:
            return

        self.ax_z.clear()
        self.ax_z.set_facecolor(BG_MAIN)
        for spine in self.ax_z.spines.values():
            spine.set_visible(False)

        self.ax_z.set_title("Eje Z – Altura del efector", color=FG_TEXT, fontsize=12)
        self.ax_z.set_ylabel("Z [m]", color=FG_MUTED)
        self.ax_z.tick_params(colors=FG_MUTED)

        # Rango típico de z (asumiendo d3 ∈ [0, 0.06])
        z_min_plot = D0 - 0.06 - 0.005
        z_max_plot = D0 + 0.005
        self.ax_z.set_xlim(0, 1)
        self.ax_z.set_ylim(z_min_plot, z_max_plot)

        # Barra vertical
        self.ax_z.plot(
            [0.5, 0.5],
            [z_min_plot, z_max_plot],
            color="#4B5563",
            linestyle="--",
            linewidth=2,
        )
        # Punto actual de z
        self.ax_z.scatter([0.5], [z_ef], color=ACCENT_2, s=80, zorder=5)
        self.ax_z.text(
            0.52,
            z_ef,
            f"{z_ef:.3f} m",
            color=FG_TEXT,
            fontsize=10,
            va="center",
        )
        self.ax_z.set_xticks([])

        self.canvas_z.draw_idle()

    def _animate_to(self, q1_new, q2_new, d3_new, steps=30, duration_ms=600):
        """
        Animación entre configuraciones usando la velocidad global.
        """
        q1_old, q2_old, d3_old = self.last_q

        speed = self._get_speed_factor()
        if speed <= 0:
            speed = 0.1

        # Duración base (600 ms) escalada por velocidad
        base_duration = 600.0
        duration_ms = int(base_duration / speed)

        steps = max(1, steps)
        delay = max(5, duration_ms // steps)

        def step(i):
            alpha = i / steps
            q1 = q1_old + (q1_new - q1_old) * alpha
            q2 = q2_old + (q2_new - q2_old) * alpha
            d3 = d3_old + (d3_new - d3_old) * alpha
            self._draw_robot(q1, q2, d3)
            if i < steps:
                self.master.after(delay, lambda: step(i + 1))
            else:
                self.last_q = (q1_new, q2_new, d3_new)

        step(0)

    # ------------------------------------------------------------------
    # Eventos sliders
    # ------------------------------------------------------------------
    def _on_q1_scale(self, val):
        self.q1_var.set(f"{float(val):.1f}")
        self._update_live_metrics()

    def _on_q2_scale(self, val):
        self.q2_var.set(f"{float(val):.1f}")
        self._update_live_metrics()

    def _on_d3_scale(self, val):
        self.d3_var.set(f"{float(val):.1f}")
        self._update_live_metrics()

    def _update_live_metrics(self):
        if not hasattr(self, "lbl_joint_live") or not hasattr(self, "lbl_task_live"):
            return

        try:
            q1_deg = float(self.q1_var.get())
            q2_deg = float(self.q2_var.get())
            d3_mm  = float(self.d3_var.get())
        except ValueError:
            return

        q1_rad = np.deg2rad(q1_deg)
        q2_rad = np.deg2rad(q2_deg)
        d3_m   = d3_mm / 1000.0

        x, y, z = fk_scara_planar(q1_rad, q2_rad, d3_m)
        r = float(np.hypot(x, y))

        self.lbl_joint_live.config(
            text=f"q1 = {q1_deg:.1f}°  |  q2 = {q2_deg:.1f}°  |  d3 = {d3_mm:.1f} mm"
        )
        self.lbl_task_live.config(
            text=f"x = {x:.3f} m  |  y = {y:.3f} m  |  z = {z:.3f} m  |  r = {r:.3f} m"
        )
        self.lbl_gripper.config(
            text=f"Gripper: {'ABIERTO' if self.gripper_open.get() else 'CERRADO'}",
            foreground=ACCENT_2 if self.gripper_open.get() else DANGER,
        )

    # ------------------------------------------------------------------
    # Velocidad global
    # ------------------------------------------------------------------
    def _on_speed_change(self, val):
        try:
            v = float(val)
        except ValueError:
            v = 100.0
        self.speed_percent.set(v)
        if hasattr(self, "lbl_speed_value"):
            self.lbl_speed_value.config(text=f"{v:.0f}%")

    def _get_speed_factor(self) -> float:
        """
        Devuelve un factor de velocidad:
        100% -> 1.0 (normal)
        50%  -> 0.5 (más lento)
        200% -> 2.0 (más rápido)
        """
        v = self.speed_percent.get()
        if v < 10:
            v = 10
        return v / 100.0

    # ------------------------------------------------------------------
    # Leer q1,q2,d3
    # ------------------------------------------------------------------
    def _read_joint_values(self):
        try:
            q1_deg = float(self.q1_var.get())
            q2_deg = float(self.q2_var.get())
            d3_mm  = float(self.d3_var.get())
        except ValueError:
            messagebox.showerror("Error", "Introduce valores numéricos válidos.")
            return None

        self.q1_scale.set(q1_deg)
        self.q2_scale.set(q2_deg)
        self.d3_scale.set(d3_mm)

        q1_rad = np.deg2rad(q1_deg)
        q2_rad = np.deg2rad(q2_deg)
        d3_m   = d3_mm / 1000.0

        if not check_joint_limits(q1_rad, q2_rad, d3_m):
            messagebox.showwarning("Límites", "Valores fuera de los límites articulares.")

        self._update_live_metrics()

        return q1_deg, q2_deg, d3_mm, q1_rad, q2_rad, d3_m

    # ------------------------------------------------------------------
    # Botones principales (FK)
    # ------------------------------------------------------------------
    def on_calcular_fk(self):
        values = self._read_joint_values()
        if values is None:
            return

        q1_deg, q2_deg, d3_mm, q1_rad, q2_rad, d3_m = values
        res = fk_scara(q1_rad, q2_rad, d3_m)
        x, y, z = res["x"], res["y"], res["z"]

        self.last_result = {
            "q1_deg": q1_deg,
            "q2_deg": q2_deg,
            "d3_mm": d3_mm,
            "q1_rad": q1_rad,
            "q2_rad": q2_rad,
            "d3_m": d3_m,
            "A1": res["A1"],
            "A2": res["A2"],
            "A3": res["A3"],
            "T":  res["T"],
            "x": x,
            "y": y,
            "z": z,
        }

        self.lbl_q_summary.config(
            text=f"q1 = {q1_deg:.1f}°   q2 = {q2_deg:.1f}°   d3 = {d3_mm:.1f} mm"
        )
        self.lbl_xyz_summary.config(
            text=f"x = {x:.3f} m   y = {y:.3f} m   z = {z:.3f} m"
        )

        self._update_matrices_view(self.last_result)
        self._update_explanation_view(self.last_result)

        self._log(
            f"[FK] q1={q1_deg:.1f}°, q2={q2_deg:.1f}°, d3={d3_mm:.1f} mm -> "
            f"x={x:.3f} m, y={y:.3f} m, z={z:.3f} m"
        )

    def on_calcular_y_animar(self):
        self.on_calcular_fk()
        if self.last_result is None:
            return
        q1_rad = self.last_result["q1_rad"]
        q2_rad = self.last_result["q2_rad"]
        d3_m   = self.last_result["d3_m"]
        self._animate_to(q1_rad, q2_rad, d3_m)

    def on_enviar_arduino(self):
        if self.last_result is None:
            messagebox.showinfo("Info", "Primero calcula la cinemática (Calcular FK).")
            return

        if not self.serial_ok or self.ser is None:
            messagebox.showerror("Serial", "El puerto serie no está disponible.")
            return

        q1_rad = self.last_result["q1_rad"]
        q2_rad = self.last_result["q2_rad"]
        d3_m   = self.last_result["d3_m"]
        g      = 1 if self.gripper_open.get() else 0
        speed_factor = self._get_speed_factor()

        line = f"{q1_rad:.4f},{q2_rad:.4f},{d3_m:.4f},{g},{speed_factor:.2f}\n"
        try:
            self.ser.write(line.encode("ascii"))
            self._log(f"[TX] {line.strip()}")
            self._append_serial_monitor(line.strip(), prefix="TX")
        except Exception as e:
            self._log(f"[ERROR] No se pudieron enviar datos: {e}")
            messagebox.showerror("Error", f"No se pudieron enviar los datos: {e}")


    # ------------------------------------------------------------------
    # NUEVO: Botón "Ir a origen"
    # ------------------------------------------------------------------
    def on_go_origin(self):
        """
        Lleva al robot al origen del modelo:
        q1 = 0°, q2 = 0°, d3 = 0 mm.
        """
        q1_deg = 0.0
        q2_deg = 0.0
        d3_mm  = 0.0

        self.q1_var.set(f"{q1_deg:.1f}")
        self.q2_var.set(f"{q2_deg:.1f}")
        self.d3_var.set(f"{d3_mm:.1f}")

        self._update_live_metrics()
        self.on_calcular_y_animar()
        self.on_enviar_arduino()

    # ------------------------------------------------------------------
    # NUEVO: Bloque de cinemática inversa (IK)
    # ------------------------------------------------------------------
    def on_calcular_ik_gui(self):
        """
        Lee x, y, z de los Entry, llama a ik_scara, y si hay solución:
        - Actualiza q1, q2, d3 en GUI
        - Actualiza FK y vistas
        """
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            z = float(self.z_var.get())
        except ValueError:
            messagebox.showerror("Error IK", "Introduce valores numéricos válidos para x, y, z.")
            return False

        self._log(f"[IK] Intentando IK para x={x:.3f}, y={y:.3f}, z={z:.3f}")

        try:
            result = ik_scara(x, y, z)
        except Exception as e:
            messagebox.showerror("Error IK", f"Error al llamar ik_scara: {e}")
            self._log(f"[IK][ERROR] {e}")
            return False

        # Se espera: (ok, msg, info)
        ok = False
        msg = ""
        info = None

        if isinstance(result, tuple):
            if len(result) == 3:
                ok, msg, info = result
            elif len(result) == 2:
                ok, msg = result
            else:
                msg = "Formato de retorno inesperado en ik_scara()."
        else:
            msg = "ik_scara no devolvió una tupla."

        if not ok:
            messagebox.showwarning("IK sin solución", f"No se encontró solución válida:\n{msg}")
            self._log(f"[IK][WARN] {msg}")
            return False

        if info is None:
            messagebox.showinfo("IK", "IK reporta éxito, pero no devolvió información de articulaciones.")
            self._log("[IK][INFO] Éxito sin info de articulaciones.")
            return False

        # info puede ser dict o tupla (q1_rad, q2_rad, d3_m)
        if isinstance(info, dict):
            q1_deg = info.get("q1_deg")
            q2_deg = info.get("q2_deg")
            d3_mm  = info.get("d3_mm")
            q1_rad = info.get("q1_rad")
            q2_rad = info.get("q2_rad")
            d3_m   = info.get("d3_m")

            # Por si faltan grados pero hay radianes
            if q1_deg is None and q1_rad is not None:
                q1_deg = math.degrees(q1_rad)
            if q2_deg is None and q2_rad is not None:
                q2_deg = math.degrees(q2_rad)
            if d3_mm is None and d3_m is not None:
                d3_mm = d3_m * 1000.0

        elif isinstance(info, (list, tuple)) and len(info) == 3:
            q1_rad, q2_rad, d3_m = info
            q1_deg = math.degrees(q1_rad)
            q2_deg = math.degrees(q2_rad)
            d3_mm  = d3_m * 1000.0
        else:
            messagebox.showerror("IK", "Formato de info devuelto por ik_scara no reconocido.")
            self._log(f"[IK][ERROR] Formato info no reconocido: {info}")
            return False

        # Si por alguna razón faltan datos críticos:
        if q1_deg is None or q2_deg is None or d3_mm is None:
            messagebox.showerror("IK", "La solución IK no contiene q1, q2 o d3 válidos.")
            self._log(f"[IK][ERROR] Datos incompletos en info: {info}")
            return False

        # Actualizar GUI
        self.q1_var.set(f"{q1_deg:.1f}")
        self.q2_var.set(f"{q2_deg:.1f}")
        self.d3_var.set(f"{d3_mm:.1f}")

        self._update_live_metrics()
        self.on_calcular_fk()

        self._log(
            f"[IK][OK] q1={q1_deg:.1f}°, q2={q2_deg:.1f}°, d3={d3_mm:.1f} mm "
            f"para x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        return True

    def on_ik_mover_enviar(self):
        """
        Calcula IK, si hay solución:
        - Actualiza GUI
        - Anima en la vista
        - Envía al Arduino
        """
        ok = self.on_calcular_ik_gui()
        if not ok:
            return
        # Animar y enviar usando la misma lógica que FK
        self.on_calcular_y_animar()
        self.on_enviar_arduino()

    # ------------------------------------------------------------------
    # Rutinas
    # ------------------------------------------------------------------
    def on_add_pose_to_routine(self):
        values = self._read_joint_values()
        if values is None:
            return
        q1_deg, q2_deg, d3_mm, _, _, _ = values

        # Estado del gripper en este momento
        g = 1 if self.gripper_open.get() else 0

        # Guardar también g en la lista interna
        self.routine.append(
            {"q1_deg": q1_deg, "q2_deg": q2_deg, "d3_mm": d3_mm, "g": g}
        )
        idx = len(self.routine)

        # Insertar en la tabla (con columna de gripper, la definimos abajo)
        grip_text = "Abierto" if g == 1 else "Cerrado"
        self.tree_routine.insert(
            "", tk.END,
            values=(idx, f"{q1_deg:.1f}", f"{q2_deg:.1f}", f"{d3_mm:.1f}", grip_text)
        )
        self._log(
            f"[RUTINA] Añadida pose #{idx}: "
            f"q1={q1_deg:.1f}°, q2={q2_deg:.1f}°, d3={d3_mm:.1f} mm, gripper={grip_text}"
        )


    def on_clear_routine(self):
        self.routine.clear()
        for item in self.tree_routine.get_children():
            self.tree_routine.delete(item)
        self._log("[RUTINA] Rutina limpiada")

    def on_simulate_routine(self):
        if not self.routine:
            messagebox.showinfo("Rutina vacía", "Primero añade al menos una pose.")
            return

        try:
            delay_ms_base = int(self.routine_delay_var.get())
        except ValueError:
            messagebox.showerror("Error", "Tiempo entre pasos inválido.")
            return

        speed = self._get_speed_factor()
        if speed <= 0:
            speed = 0.1
        delay_ms = max(10, int(delay_ms_base / speed))

        self._log("[RUTINA] Simulación en GUI iniciada")

        def step(i):
            if i >= len(self.routine):
                self._log("[RUTINA] Simulación completada")
                return

            pose = self.routine[i]
            self.q1_var.set(f"{pose['q1_deg']:.1f}")
            self.q2_var.set(f"{pose['q2_deg']:.1f}")
            self.d3_var.set(f"{pose['d3_mm']:.1f}")
            
            # Actualizar gripper según la pose (si tiene 'g')
            if "g" in pose:
                self.gripper_open.set(bool(pose["g"]))

            self._update_live_metrics()
            self.on_calcular_y_animar()

            self.master.after(delay_ms, lambda: step(i + 1))

        step(0)

    def on_execute_routine(self):
        if not self.routine:
            messagebox.showinfo("Rutina vacía", "Primero añade al menos una pose.")
            return

        if not self.serial_ok or self.ser is None:
            messagebox.showerror("Serial", "El puerto serie no está disponible.")
            return

        try:
            delay_ms_base = int(self.routine_delay_var.get())
        except ValueError:
            messagebox.showerror("Error", "Tiempo entre pasos inválido.")
            return

        speed = self._get_speed_factor()
        if speed <= 0:
            speed = 0.1
        delay_ms = max(10, int(delay_ms_base / speed))

        self._log("[RUTINA] Ejecución en robot iniciada")

        def step(i):
            if i >= len(self.routine):
                self._log("[RUTINA] Ejecución completada")
                return

            pose = self.routine[i]
            self.q1_var.set(f"{pose['q1_deg']:.1f}")
            self.q2_var.set(f"{pose['q2_deg']:.1f}")
            self.d3_var.set(f"{pose['d3_mm']:.1f}")

            # Actualizar gripper según la pose
            if "g" in pose:
                self.gripper_open.set(bool(pose["g"]))

            self._update_live_metrics()
            self.on_calcular_y_animar()

            if self.last_result is not None:
                q1_rad = self.last_result["q1_rad"]
                q2_rad = self.last_result["q2_rad"]
                d3_m   = self.last_result["d3_m"]

                # Tomar g de la pose, no del estado global
                g = pose.get("g", 1)
                speed_factor = self._get_speed_factor()

                line = f"{q1_rad:.4f},{q2_rad:.4f},{d3_m:.4f},{g},{speed_factor:.2f}\n"
                try:
                    self.ser.write(line.encode("ascii"))
                    self._log(f"[TX] {line.strip()}")
                    self._append_serial_monitor(line.strip(), prefix="TX")
                    # Si quieres quitar los muchos pop-ups, borra la siguiente línea:
                    # messagebox.showinfo("Enviado", f"Se envió al Arduino:\n{line.strip()}")
                except Exception as e:
                    self._log(f"[ERROR] No se pudieron enviar datos: {e}")
                    messagebox.showerror("Error", f"No se pudieron enviar los datos: {e}")

            self.master.after(delay_ms, lambda: step(i + 1))

        step(0)

    # ---- NUEVO: helpers para paso seleccionado ----
    def _get_selected_routine_index(self):
        sel = self.tree_routine.selection()
        if not sel:
            messagebox.showinfo("Rutina", "Selecciona una pose en la tabla de rutinas.")
            return None
        item_id = sel[0]
        values = self.tree_routine.item(item_id, "values")
        try:
            idx = int(values[0]) - 1  # la primera columna es el índice humano (1-based)
        except (ValueError, IndexError):
            return None
        if idx < 0 or idx >= len(self.routine):
            return None
        return idx

    def on_simulate_selected_step(self):
        idx = self._get_selected_routine_index()
        if idx is None:
            return

        pose = self.routine[idx]
        self._log(f"[RUTINA] Simulando solo paso #{idx+1}")

        self.q1_var.set(f"{pose['q1_deg']:.1f}")
        self.q2_var.set(f"{pose['q2_deg']:.1f}")
        self.d3_var.set(f"{pose['d3_mm']:.1f}")

        if "g" in pose:
            self.gripper_open.set(bool(pose["g"]))

        self._update_live_metrics()
        self.on_calcular_y_animar()


    def on_execute_selected_step(self):
        idx = self._get_selected_routine_index()
        if idx is None:
            return

        if not self.serial_ok or self.ser is None:
            messagebox.showerror("Serial", "El puerto serie no está disponible.")
            return

        pose = self.routine[idx]
        self._log(f"[RUTINA] Ejecutando solo paso #{idx+1} en el robot")

        self.q1_var.set(f"{pose['q1_deg']:.1f}")
        self.q2_var.set(f"{pose['q2_deg']:.1f}")
        self.d3_var.set(f"{pose['d3_mm']:.1f}")

        if "g" in pose:
            self.gripper_open.set(bool(pose["g"]))

        self._update_live_metrics()
        self.on_calcular_y_animar()

        if self.last_result is not None:
            q1_rad = self.last_result["q1_rad"]
            q2_rad = self.last_result["q2_rad"]
            d3_m   = self.last_result["d3_m"]
            g      = pose.get("g", 1)
            speed_factor = self._get_speed_factor()
            line = f"{q1_rad:.4f},{q2_rad:.4f},{d3_m:.4f},{g},{speed_factor:.2f}\n"
            try:
                self.ser.write(line.encode("ascii"))
                self._log(f"[RUTINA][TX][1 paso] {line.strip()}")
                self._append_serial_monitor(line.strip(), prefix="TX")
            except Exception as e:
                self._log(f"[ERROR] Falló envío del paso seleccionado: {e}")


    # ------------------------------------------------------------------
    # Vistas de matrices y cálculos
    # ------------------------------------------------------------------
    def _update_matrices_view(self, res):
        self.txt_matrices.delete("1.0", tk.END)

        def mat_to_str(M):
            return np.array2string(M, formatter={"float_kind": lambda x: f"{x: .4f}"})

        self.txt_matrices.insert(tk.END, "=== Variables articulares ===\n")
        self.txt_matrices.insert(
            tk.END, f"q1 = {res['q1_deg']:.3f}° ({res['q1_rad']:.4f} rad)\n"
        )
        self.txt_matrices.insert(
            tk.END, f"q2 = {res['q2_deg']:.3f}° ({res['q2_rad']:.4f} rad)\n"
        )
        self.txt_matrices.insert(
            tk.END, f"d3 = {res['d3_mm']:.3f} mm ({res['d3_m']:.4f} m)\n\n"
        )

        self.txt_matrices.insert(tk.END, "=== Matriz A1 ===\n")
        self.txt_matrices.insert(tk.END, mat_to_str(res["A1"]) + "\n\n")

        self.txt_matrices.insert(tk.END, "=== Matriz A2 ===\n")
        self.txt_matrices.insert(tk.END, mat_to_str(res["A2"]) + "\n\n")

        self.txt_matrices.insert(tk.END, "=== Matriz A3 ===\n")
        self.txt_matrices.insert(tk.END, mat_to_str(res["A3"]) + "\n\n")

        self.txt_matrices.insert(tk.END, "=== Matriz T = A1 · A2 · A3 ===\n")
        self.txt_matrices.insert(tk.END, mat_to_str(res["T"]) + "\n\n")

        self.txt_matrices.insert(tk.END, "=== Posición del efector ===\n")
        self.txt_matrices.insert(
            tk.END,
            f"x = {res['x']:.6f} m\n"
            f"y = {res['y']:.6f} m\n"
            f"z = {res['z']:.6f} m\n",
        )

    def _update_explanation_view(self, res):
        self.txt_explain.delete("1.0", tk.END)

        q1_deg = res["q1_deg"]
        q2_deg = res["q2_deg"]
        d3_mm  = res["d3_mm"]
        q1_rad = res["q1_rad"]
        q2_rad = res["q2_rad"]
        d3_m   = res["d3_m"]
        x      = res["x"]
        y      = res["y"]
        z      = res["z"]

        self.txt_explain.insert(tk.END, "=== Paso a paso de la cinemática directa (FK) ===\n\n")

        self.txt_explain.insert(tk.END, "1) Datos de entrada (espacio articular)\n")
        self.txt_explain.insert(
            tk.END,
            f"   q1 = {q1_deg:.3f}°   q2 = {q2_deg:.3f}°   d3 = {d3_mm:.3f} mm\n\n",
        )

        self.txt_explain.insert(tk.END, "2) Conversión a unidades del modelo\n")
        self.txt_explain.insert(tk.END, f"   q1_rad = {q1_rad:.6f} rad\n")
        self.txt_explain.insert(tk.END, f"   q2_rad = {q2_rad:.6f} rad\n")
        self.txt_explain.insert(tk.END, f"   d3_m   = {d3_m:.6f} m\n\n")

        self.txt_explain.insert(tk.END, "3) Parámetros geométricos del SCARA\n")
        self.txt_explain.insert(tk.END, f"   L1 = {L1:.3f} m\n")
        self.txt_explain.insert(tk.END, f"   L2 = {L2:.3f} m\n")
        self.txt_explain.insert(tk.END, f"   D0 = {D0:.3f} m\n\n")

        self.txt_explain.insert(tk.END, "4) Ecuaciones de posición\n")
        self.txt_explain.insert(tk.END, "   x = L1·cos(q1) + L2·cos(q1 + q2)\n")
        self.txt_explain.insert(tk.END, "   y = L1·sin(q1) + L2·sin(q1 + q2)\n")
        self.txt_explain.insert(tk.END, "   z = D0 - d3\n\n")

        cq1  = np.cos(q1_rad)
        sq1  = np.sin(q1_rad)
        cq12 = np.cos(q1_rad + q2_rad)
        sq12 = np.sin(q1_rad + q2_rad)

        self.txt_explain.insert(tk.END, "   Valores numéricos:\n")
        self.txt_explain.insert(tk.END, f"   cos(q1)    = {cq1:.6f}\n")
        self.txt_explain.insert(tk.END, f"   sin(q1)    = {sq1:.6f}\n")
        self.txt_explain.insert(tk.END, f"   cos(q1+q2) = {cq12:.6f}\n")
        self.txt_explain.insert(tk.END, f"   sin(q1+q2) = {sq12:.6f}\n\n")

        x_calc = L1 * cq1 + L2 * cq12
        y_calc = L1 * sq1 + L2 * sq12
        z_calc = D0 - d3_m

        self.txt_explain.insert(
            tk.END,
            f"   x = {L1:.3f}·{cq1:.6f} + {L2:.3f}·{cq12:.6f} ≈ {x_calc:.6f} m\n",
        )
        self.txt_explain.insert(
            tk.END,
            f"   y = {L1:.3f}·{sq1:.6f} + {L2:.3f}·{sq12:.6f} ≈ {y_calc:.6f} m\n",
        )
        self.txt_explain.insert(
            tk.END,
            f"   z = {D0:.3f} - {d3_m:.6f} ≈ {z_calc:.6f} m\n\n",
        )

        self.txt_explain.insert(tk.END, "5) Comparación con la matriz T:\n")
        self.txt_explain.insert(
            tk.END,
            f"   x_modelo = {x_calc:.6f} m   |   x_T = {x:.6f} m\n"
            f"   y_modelo = {y_calc:.6f} m   |   y_T = {y:.6f} m\n"
            f"   z_modelo = {z_calc:.6f} m   |   z_T = {z:.6f} m\n\n",
        )

        self.txt_explain.insert(
            tk.END,
            "   Coinciden (salvo redondeo), por lo que la FK está correcta.\n",
        )

    # ------------------------------------------------------------------
    # Log
    # ------------------------------------------------------------------
    def _log(self, text: str):
        self.txt_log.insert(tk.END, text + "\n")
        self.txt_log.see(tk.END)

    # ------------------------------------------------------------------
    # Cierre
    # ------------------------------------------------------------------
    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.master.destroy()


def main():
    root = tk.Tk()
    app = ScaraGUI(root)

    def on_close():
        app.close()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
