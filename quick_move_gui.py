"""Quick Move control interface for myCobot manipulation project."""

from __future__ import annotations

import json
import os
import csv
import threading
import time
import traceback
import tkinter as tk
from copy import deepcopy
from datetime import datetime
from math import ceil, isfinite, sqrt
from pathlib import Path
from tkinter import filedialog, ttk
from typing import Any, Callable

try:
    from pymycobot import MyCobot280
except Exception:  # pragma: no cover - handled at runtime on systems without hardware libs.
    MyCobot280 = None  # type: ignore[assignment]


class QuickMoveGUI(tk.Tk):
    BG = "#E7E7E7"
    BTN_BG = "#DDE1E6"
    VAL_BG = "#D2D6DC"
    VAL_FG = "#4D545D"
    READ_BG = "#E6F1DD"
    READ_FG = "#63A548"
    SETTINGS_FILE = "quick_move_settings.json"
    DEFAULT_PORT = os.getenv("MYCOBOT_PORT", "/dev/ttyACM0")
    DEFAULT_BAUD = int(os.getenv("MYCOBOT_BAUD", "115200"))
    SERIAL_PORT_OPTIONS = ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "COM3", "COM4", "COM5")
    BAUDRATE_OPTIONS = ("9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600", "1000000")
    SPEED = 50
    LINEAR_MODE = 0
    VAC_PIN_ON = 2
    VAC_PIN_OFF = 5
    XYZ_INTERP_STEP_MM = 5
    XYZ_INTERP_SETTLE_S = 0.08
    XYZ_STREAM_HZ = 30.0
    XYZ_MIN_STREAM_STEPS = 12
    XYZ_ARRIVE_TOL_MM = 8.0
    XYZ_MIN_PROGRESS_MM = 2.0
    XYZ_MM_PER_SEC_EST = 8.0
    XYZ_MAX_WAIT_S = 40.0
    HOME_RPY_ARRIVE_TOL_DEG = 6.0
    MOTION_READY_SETTLE_S = 0.5
    # Sequence/record moves need looser settle tolerance due controller quantization/backlash.
    RECORD_MOVE_ARRIVE_TOL_MM = 6.0
    RECORD_MOVE_RPY_ARRIVE_TOL_DEG = 8.0
    RECORD_MOVE_POLL_S = 0.03
    RECORD_MAX_RATE_FALLBACK_SLEEP_S = 0.002
    CLOSEST_FALLBACK_ENABLED = True
    CLOSEST_FALLBACK_STEPS = 30
    CLOSEST_FALLBACK_TRY_LIMIT = 4
    CLOSEST_FALLBACK_MIN_PROGRESS_MM = 1.0
    CLOSEST_FALLBACK_MIN_PROGRESS_DEG = 1.0
    ZERO_JOINT_STEP_DEG = 3.0
    ZERO_ARRIVE_TOL_DEG = 3.0
    HOME_X = 150.0
    HOME_Y = -60.0
    HOME_Z = 290.0
    HOME_RX = -90.0
    HOME_RY = 0.0
    HOME_RZ = -90.0
    JOINT_STEP = 1.0
    COORD_STEPS = {
        "x": 5.0,
        "y": 5.0,
        "z": 5.0,
        "rx": 1.0,
        "ry": 1.0,
        "rz": 1.0,
    }
    # MyCobot280 limits used by pymycobot validation (prevents "error on index X").
    COORD_MIN = [-281.45, -281.45, -70.0, -180.0, -180.0, -180.0]
    COORD_MAX = [281.45, 281.45, 412.67, 180.0, 180.0, 180.0]
    JOINT_ORDER = ("J1", "J2", "J3", "J4", "J5", "J6")
    COORD_ORDER = ("x", "y", "z", "rx", "ry", "rz")
    HOME_DEFAULTS = (150.0, -60.0, 290.0, -90.0, 0.0, -90.0)
    TOOL_LENGTH_DEFAULT_MM = 0.0

    def __init__(self) -> None:
        super().__init__()
        self.title("Quick Move")
        self.configure(bg=self.BG)
        self.resizable(True, True)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.bind("<F11>", self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)

        self.settings_path = Path(__file__).resolve().with_name(self.SETTINGS_FILE)
        self.serial_port = self.DEFAULT_PORT
        self.serial_baud = self.DEFAULT_BAUD
        self._set_home_pose(list(self.HOME_DEFAULTS))
        self.tool_length_mm = float(self.TOOL_LENGTH_DEFAULT_MM)
        self.named_positions: dict[str, list[float]] = {}
        self._load_settings_file()

        self.settings_port_var = tk.StringVar(value=self.serial_port)
        self.settings_baud_var = tk.StringVar(value=str(self.serial_baud))
        self.settings_home_vars = {
            axis: tk.StringVar(value=self._format_value(value))
            for axis, value in zip(self.COORD_ORDER, self._get_home_pose())
        }
        self.settings_tool_length_var = tk.StringVar(value=self._format_value(self.tool_length_mm))
        self.settings_tol_record_xyz_var = tk.StringVar(value=self._format_value(self.RECORD_MOVE_ARRIVE_TOL_MM))
        self.settings_tol_record_rpy_var = tk.StringVar(value=self._format_value(self.RECORD_MOVE_RPY_ARRIVE_TOL_DEG))
        self.settings_tol_general_xyz_var = tk.StringVar(value=self._format_value(self.XYZ_ARRIVE_TOL_MM))
        self.settings_tol_general_rpy_var = tk.StringVar(value=self._format_value(self.HOME_RPY_ARRIVE_TOL_DEG))
        self.position_name_var = tk.StringVar(value="")
        self.record_named_pose_var = tk.StringVar(value="")
        self.record_csv_dir_var = tk.StringVar(value=str(self.settings_path.parent))
        self.record_csv_name_var = tk.StringVar(value="motion_record.csv")
        self.record_speed_var = tk.StringVar(value=str(self.SPEED))
        self.record_start_step_var = tk.StringVar(value="1")
        self.record_end_step_var = tk.StringVar(value="")
        self.record_wait_ms_var = tk.StringVar(value="500")
        self.record_sequence_file_var = tk.StringVar(value=str(self.settings_path.parent / "motion_sequence.json"))
        self.joint_values = {
            "J1": tk.DoubleVar(value=None),
            "J2": tk.DoubleVar(value=None),
            "J3": tk.DoubleVar(value=None),
            "J4": tk.DoubleVar(value=None),
            "J5": tk.DoubleVar(value=None),
            "J6": tk.DoubleVar(value=None),
        }
        self.coord_values = {
            "x": tk.DoubleVar(value=None),
            "y": tk.DoubleVar(value=None),
            "z": tk.DoubleVar(value=None),
            "rx": tk.DoubleVar(value=None),
            "ry": tk.DoubleVar(value=None),
            "rz": tk.DoubleVar(value=None),
        }
        self.target_pose_vars = {
            axis: tk.StringVar(value=self._format_value(self.coord_values[axis].get()))
            for axis in self.COORD_ORDER
        }
        self.named_positions_listbox: tk.Listbox | None = None
        self.record_named_pose_combo: ttk.Combobox | None = None
        self.sequence_listbox: tk.Listbox | None = None
        self.sequence_pose_vars = {axis: tk.StringVar(value="") for axis in self.COORD_ORDER}
        self.sequence_pose_entries: dict[str, tk.Entry] = {}
        self.sequence_pose_apply_btn: tk.Button | None = None
        self.sequence_pose_reset_btn: tk.Button | None = None
        self.sequence_step_clipboard: dict[str, Any] | None = None
        self.sequence_steps: list[dict[str, Any]] = []
        self.mc: Any | None = None
        self.busy_event = threading.Event()
        self.abort_event = threading.Event()
        self.status_var = tk.StringVar(
            value=f"Ready - click a control to connect ({self.serial_port} @ {self.serial_baud})"
        )
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build_ui()
        self.after(0, self._maximize_window)

    def _build_ui(self) -> None:
        notebook = ttk.Notebook(self)
        notebook.grid(row=0, column=0, sticky="nsew")

        def make_centered_tab(title: str) -> tk.Frame:
            page = tk.Frame(notebook, bg=self.BG)
            page.rowconfigure(0, weight=1)
            page.columnconfigure(0, weight=1)
            content = tk.Frame(page, bg=self.BG, padx=10, pady=8)
            # Let each tab content expand with the window.
            content.grid(row=0, column=0, sticky="nsew")
            notebook.add(page, text=title)
            return content

        main = make_centered_tab("Quick Move")
        settings = make_centered_tab("Settings")
        record = make_centered_tab("Record Motion")

        tk.Label(
            main,
            text="Quick Move",
            font=("Times New Roman", 20, "bold"),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=0, column=0, columnspan=2, sticky="w")

        tk.Label(
            main,
            text="Joints Control:",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=1, column=0, sticky="w", pady=(10, 6))

        tk.Button(
            main,
            text="Read Angles",
            command=self._read_angles,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=16,
            pady=3,
            font=("Arial", 11),
        ).grid(row=1, column=1, sticky="w", pady=(10, 6))

        joints_grid = tk.Frame(main, bg=self.BG)
        joints_grid.grid(row=2, column=0, columnspan=2, sticky="w")
        left_joint_names = ("J1", "J3", "J5")
        right_joint_names = ("J2", "J4", "J6")
        for row, name in enumerate(left_joint_names):
            self._build_value_control(
                joints_grid,
                name,
                self.joint_values[name],
                on_minus=lambda n=name: self._step_joint(n, -self.JOINT_STEP),
                on_plus=lambda n=name: self._step_joint(n, self.JOINT_STEP),
                on_set=lambda value, n=name: self._set_joint(n, value),
                row=row,
                col=0,
            )
        for row, name in enumerate(right_joint_names):
            self._build_value_control(
                joints_grid,
                name,
                self.joint_values[name],
                on_minus=lambda n=name: self._step_joint(n, -self.JOINT_STEP),
                on_plus=lambda n=name: self._step_joint(n, self.JOINT_STEP),
                on_set=lambda value, n=name: self._set_joint(n, value),
                row=row,
                col=2,
            )

        tk.Label(
            main,
            text="Coordination Control:",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=3, column=0, sticky="w", pady=(12, 6))

        tk.Button(
            main,
            text="Read Coords",
            command=self._read_coords,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=16,
            pady=3,
            font=("Arial", 11),
        ).grid(row=3, column=1, sticky="w", pady=(12, 6))

        coords_grid = tk.Frame(main, bg=self.BG)
        coords_grid.grid(row=4, column=0, columnspan=2, sticky="w")
        left_coord_names = ("x", "y", "z")
        right_coord_names = ("rx", "ry", "rz")
        for row, name in enumerate(left_coord_names):
            step = self.COORD_STEPS[name]
            self._build_value_control(
                coords_grid,
                name,
                self.coord_values[name],
                on_minus=lambda n=name, d=step: self._step_coord(n, -d),
                on_plus=lambda n=name, d=step: self._step_coord(n, d),
                on_set=lambda value, n=name: self._set_coord(n, value),
                row=row,
                col=0,
            )
        for row, name in enumerate(right_coord_names):
            step = self.COORD_STEPS[name]
            self._build_value_control(
                coords_grid,
                name,
                self.coord_values[name],
                on_minus=lambda n=name, d=step: self._step_coord(n, -d),
                on_plus=lambda n=name, d=step: self._step_coord(n, d),
                on_set=lambda value, n=name: self._set_coord(n, value),
                row=row,
                col=2,
            )

        tk.Label(
            main,
            text="Target Pose:",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=5, column=0, sticky="w", pady=(12, 6))

        target_box = tk.Frame(main, bg=self.BG)
        target_box.grid(row=6, column=0, columnspan=2, sticky="w")
        for row, axes in enumerate((("x", "y", "z"), ("rx", "ry", "rz"))):
            for col, axis in enumerate(axes):
                tk.Label(
                    target_box,
                    text=axis,
                    font=("Times New Roman", 13),
                    bg=self.BG,
                    fg="#4D4D4D",
                    width=2,
                ).grid(row=row, column=col * 2, sticky="e", padx=(6, 3), pady=1)
                tk.Entry(
                    target_box,
                    textvariable=self.target_pose_vars[axis],
                    bg=self.VAL_BG,
                    fg=self.VAL_FG,
                    width=8,
                    justify="center",
                    font=("Arial", 11),
                    relief="flat",
                    bd=0,
                    highlightthickness=0,
                    insertbackground=self.VAL_FG,
                ).grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 8), ipady=2, pady=1)

        tk.Button(
            target_box,
            text="Use Current Pose",
            command=self._load_target_pose_from_display,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 10),
        ).grid(row=0, column=6, rowspan=2, sticky="w", padx=(8, 6))

        tk.Button(
            target_box,
            text="Move To Pose",
            command=self._move_to_pose,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=14,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=7, rowspan=2, sticky="w")

        tk.Label(
            target_box,
            text="Name:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=2, column=0, sticky="e", padx=(6, 3), pady=(8, 2))
        tk.Entry(
            target_box,
            textvariable=self.position_name_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=22,
            justify="left",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=2, column=1, columnspan=3, sticky="w", padx=(0, 8), ipady=2, pady=(8, 2))

        tk.Button(
            target_box,
            text="Save Named Pose",
            command=self._save_named_position,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=2, column=6, columnspan=2, sticky="w", pady=(8, 2))

        self.named_positions_listbox = tk.Listbox(
            target_box,
            width=48,
            height=5,
            font=("Arial", 10),
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            relief="flat",
            highlightthickness=1,
            highlightbackground="#B8BEC7",
            selectbackground="#B8BEC7",
            selectforeground="#3F4650",
            exportselection=False,
        )
        self.named_positions_listbox.grid(row=3, column=0, columnspan=6, rowspan=2, sticky="w", padx=(6, 8), pady=(2, 2))
        self.named_positions_listbox.bind("<Double-Button-1>", self._load_selected_named_position)

        tk.Button(
            target_box,
            text="Load Selected",
            command=self._load_selected_named_position,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=3, column=6, columnspan=2, sticky="w", pady=(2, 2))

        tk.Button(
            target_box,
            text="Delete Selected",
            command=self._delete_selected_named_position,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=4, column=6, columnspan=2, sticky="w", pady=(2, 2))

        tk.Button(
            target_box,
            text="Move Selected",
            command=self._move_selected_named_position,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=5, column=6, columnspan=2, sticky="w", pady=(2, 2))

        tk.Label(
            target_box,
            text="Hand Guide:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=5, column=0, sticky="e", padx=(6, 3), pady=(4, 2))

        tk.Button(
            target_box,
            text="Free Joints",
            command=self._free_joints,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=5, column=1, sticky="w", pady=(4, 2))

        tk.Button(
            target_box,
            text="Lock Joints",
            command=self._lock_joints,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=5, column=2, sticky="w", padx=(0, 6), pady=(4, 2))

        tk.Button(
            target_box,
            text="Save Current As Named",
            command=self._save_current_pose_as_named,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=5, column=3, columnspan=3, sticky="w", pady=(4, 2))

        self._refresh_named_positions_list()

        tk.Label(
            main,
            text="Reset Positions:",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=7, column=0, sticky="w", pady=(12, 6))

        reset_box = tk.Frame(main, bg=self.BG)
        reset_box.grid(row=8, column=0, columnspan=2, sticky="w")

        tk.Button(
            reset_box,
            text="Zero Joints",
            command=self._go_zero_joints,
            bg=self.BTN_BG,
            fg="#606870",
            activebackground=self.BTN_BG,
            activeforeground="#606870",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=0, sticky="w", padx=(6, 8))

        tk.Button(
            reset_box,
            text="Go Home",
            command=self._go_home_position,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=1, sticky="w")

        tk.Label(
            main,
            text="Vacuum Control:",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=9, column=0, sticky="w", pady=(12, 6))

        vacuum_box = tk.Frame(main, bg=self.BG)
        vacuum_box.grid(row=10, column=0, columnspan=2, sticky="w")

        tk.Button(
            vacuum_box,
            text="Vacuum ON",
            command=lambda: self._set_vacuum(True),
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=16,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=0, sticky="w", padx=(6, 8))

        tk.Button(
            vacuum_box,
            text="Vacuum OFF",
            command=lambda: self._set_vacuum(False),
            bg=self.BTN_BG,
            fg="#606870",
            activebackground=self.BTN_BG,
            activeforeground="#606870",
            relief="groove",
            bd=1,
            padx=16,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=1, sticky="w")

        tk.Button(
            vacuum_box,
            text="Abort Motion",
            command=self._abort_motion,
            bg="#F4DDDD",
            fg="#9A4040",
            activebackground="#F4DDDD",
            activeforeground="#9A4040",
            relief="groove",
            bd=1,
            padx=16,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=2, sticky="w", padx=(8, 0))

        status = tk.Label(main, textvariable=self.status_var, bg=self.BG, fg="#5D646D", font=("Arial", 9))
        status.grid(row=11, column=0, columnspan=2, sticky="w", pady=(10, 0))
        self._build_settings_tab(settings)
        self._build_record_tab(record)

    def _build_settings_tab(self, parent: tk.Widget) -> None:
        tk.Label(
            parent,
            text="Settings",
            font=("Times New Roman", 20, "bold"),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=0, column=0, columnspan=4, sticky="w")

        tk.Label(
            parent,
            text="Serial Connection",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(10, 6))

        tk.Label(
            parent,
            text="Port:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=2, column=0, sticky="e", padx=(6, 4), pady=2)
        ttk.Combobox(
            parent,
            textvariable=self.settings_port_var,
            values=self.SERIAL_PORT_OPTIONS,
            width=18,
            state="normal",
        ).grid(row=2, column=1, sticky="w", padx=(0, 14), ipady=2, pady=2)

        tk.Label(
            parent,
            text="Baudrate:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=2, column=2, sticky="e", padx=(6, 4), pady=2)
        ttk.Combobox(
            parent,
            textvariable=self.settings_baud_var,
            values=self.BAUDRATE_OPTIONS,
            width=12,
            state="normal",
        ).grid(row=2, column=3, sticky="w", padx=(0, 6), pady=2)

        tk.Label(
            parent,
            text="Home Pose",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(12, 6))

        home_box = tk.Frame(parent, bg=self.BG)
        home_box.grid(row=4, column=0, columnspan=4, sticky="w")
        for row, axes in enumerate((("x", "y", "z"), ("rx", "ry", "rz"))):
            for col, axis in enumerate(axes):
                tk.Label(
                    home_box,
                    text=axis,
                    font=("Times New Roman", 13),
                    bg=self.BG,
                    fg="#4D4D4D",
                    width=2,
                ).grid(row=row, column=col * 2, sticky="e", padx=(6, 3), pady=1)
                tk.Entry(
                    home_box,
                    textvariable=self.settings_home_vars[axis],
                    bg=self.VAL_BG,
                    fg=self.VAL_FG,
                    width=8,
                    justify="center",
                    font=("Arial", 11),
                    relief="flat",
                    bd=0,
                    highlightthickness=0,
                    insertbackground=self.VAL_FG,
                ).grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 8), ipady=2, pady=1)

        tk.Button(
            parent,
            text="Use Current Pose As Home",
            command=self._load_home_pose_from_display,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 10),
        ).grid(row=5, column=0, columnspan=2, sticky="w", padx=(6, 8), pady=(10, 0))

        tk.Label(
            parent,
            text="Tool",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=6, column=0, columnspan=4, sticky="w", pady=(12, 6))

        tk.Label(
            parent,
            text="Length (mm):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=7, column=0, sticky="e", padx=(6, 4), pady=2)
        tk.Entry(
            parent,
            textvariable=self.settings_tool_length_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=10,
            justify="center",
            font=("Arial", 11),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=7, column=1, sticky="w", ipady=2, pady=2)

        tk.Label(
            parent,
            text="In-Position Tolerances",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=8, column=0, columnspan=4, sticky="w", pady=(12, 6))

        tol_box = tk.Frame(parent, bg=self.BG)
        tol_box.grid(row=9, column=0, columnspan=4, sticky="w")

        tk.Label(
            tol_box,
            text="Record XYZ (mm):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=0, column=0, sticky="e", padx=(6, 4), pady=2)
        tk.Entry(
            tol_box,
            textvariable=self.settings_tol_record_xyz_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 11),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=0, column=1, sticky="w", ipady=2, pady=2, padx=(0, 12))

        tk.Label(
            tol_box,
            text="Record RPY (deg):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=0, column=2, sticky="e", padx=(6, 4), pady=2)
        tk.Entry(
            tol_box,
            textvariable=self.settings_tol_record_rpy_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 11),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=0, column=3, sticky="w", ipady=2, pady=2)

        tk.Label(
            tol_box,
            text="General XYZ (mm):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=1, column=0, sticky="e", padx=(6, 4), pady=2)
        tk.Entry(
            tol_box,
            textvariable=self.settings_tol_general_xyz_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 11),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=1, column=1, sticky="w", ipady=2, pady=2, padx=(0, 12))

        tk.Label(
            tol_box,
            text="General RPY (deg):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            anchor="e",
        ).grid(row=1, column=2, sticky="e", padx=(6, 4), pady=2)
        tk.Entry(
            tol_box,
            textvariable=self.settings_tol_general_rpy_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 11),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=1, column=3, sticky="w", ipady=2, pady=2)

        tk.Button(
            parent,
            text="Apply + Save Settings",
            command=self._apply_and_save_settings,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 10),
        ).grid(row=10, column=0, columnspan=2, sticky="w", padx=(6, 0), pady=(10, 0))

        tk.Label(
            parent,
            text=f"File: {self.settings_path}",
            font=("Arial", 9),
            bg=self.BG,
            fg="#5D646D",
        ).grid(row=11, column=0, columnspan=4, sticky="w", pady=(10, 0))

    def _build_record_tab(self, parent: tk.Widget) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        tk.Label(
            parent,
            text="Record Motion",
            font=("Times New Roman", 20, "bold"),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=0, column=0, sticky="w")

        sequence_box = tk.LabelFrame(
            parent,
            text="Sequence Builder",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
            padx=8,
            pady=6,
        )
        sequence_box.grid(row=1, column=0, sticky="nsew", padx=(6, 6), pady=(10, 6))
        sequence_box.columnconfigure(0, weight=0)
        sequence_box.columnconfigure(1, weight=1)
        sequence_box.columnconfigure(2, weight=0)
        sequence_box.columnconfigure(3, weight=0)
        sequence_box.columnconfigure(4, weight=0)
        sequence_box.rowconfigure(2, weight=1)
        sequence_box.rowconfigure(3, weight=0)

        tk.Label(
            sequence_box,
            text="Named pose:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=0, column=0, sticky="e", padx=(2, 4), pady=2)
        self.record_named_pose_combo = ttk.Combobox(
            sequence_box,
            textvariable=self.record_named_pose_var,
            values=[],
            state="normal",
        )
        self.record_named_pose_combo.grid(row=0, column=1, sticky="ew", padx=(0, 8), pady=2)

        tk.Button(
            sequence_box,
            text="Add Move",
            command=self._add_sequence_move_named,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 10),
        ).grid(row=0, column=2, sticky="w", padx=(0, 6), pady=2)

        tk.Button(
            sequence_box,
            text="Add Home",
            command=self._add_sequence_home,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 10),
        ).grid(row=0, column=3, sticky="w", padx=(0, 6), pady=2)

        tk.Label(
            sequence_box,
            text="Wait (ms):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=1, column=0, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            sequence_box,
            textvariable=self.record_wait_ms_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=10,
            justify="center",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=1, column=1, sticky="w", ipady=2, pady=2)

        tk.Button(
            sequence_box,
            text="Add Wait",
            command=self._add_sequence_wait,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=1, column=2, sticky="w", padx=(0, 6), pady=2)

        tk.Button(
            sequence_box,
            text="Add Vacuum ON",
            command=lambda: self._add_sequence_vacuum(True),
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=1, column=3, sticky="w", padx=(0, 6), pady=2)

        tk.Button(
            sequence_box,
            text="Add Vacuum OFF",
            command=lambda: self._add_sequence_vacuum(False),
            bg=self.BTN_BG,
            fg="#606870",
            activebackground=self.BTN_BG,
            activeforeground="#606870",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=1, column=4, sticky="w", pady=2)

        self.sequence_listbox = tk.Listbox(
            sequence_box,
            height=10,
            font=("Arial", 10),
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            relief="flat",
            highlightthickness=1,
            highlightbackground="#B8BEC7",
            selectbackground="#B8BEC7",
            selectforeground="#3F4650",
            exportselection=False,
        )
        self.sequence_listbox.grid(row=2, column=0, columnspan=4, sticky="nsew", padx=(2, 8), pady=(6, 2))
        self.sequence_listbox.bind("<<ListboxSelect>>", lambda _event: self._sync_sequence_pose_editor_from_selection())
        self.sequence_listbox.bind("<Control-c>", self._on_sequence_copy_shortcut)
        self.sequence_listbox.bind("<Control-C>", self._on_sequence_copy_shortcut)
        self.sequence_listbox.bind("<Control-v>", self._on_sequence_paste_shortcut)
        self.sequence_listbox.bind("<Control-V>", self._on_sequence_paste_shortcut)

        sequence_actions = tk.Frame(sequence_box, bg=self.BG)
        sequence_actions.grid(row=2, column=4, sticky="nsw", pady=(6, 2))

        tk.Button(
            sequence_actions,
            text="Move Up",
            command=self._move_sequence_up,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=0, column=0, sticky="ew", pady=(0, 4))

        tk.Button(
            sequence_actions,
            text="Move Down",
            command=self._move_sequence_down,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=1, column=0, sticky="ew", pady=4)

        tk.Button(
            sequence_actions,
            text="Delete Selected",
            command=self._delete_sequence_selected,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=2, column=0, sticky="ew", pady=4)

        tk.Button(
            sequence_actions,
            text="Clear Sequence",
            command=self._clear_sequence,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=3, column=0, sticky="ew", pady=(4, 0))

        tk.Button(
            sequence_actions,
            text="Copy Selected",
            command=self._copy_sequence_selected,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=4, column=0, sticky="ew", pady=(4, 0))

        tk.Button(
            sequence_actions,
            text="Paste",
            command=self._paste_sequence_step,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=5, column=0, sticky="ew", pady=(4, 0))

        pose_editor = tk.LabelFrame(
            sequence_box,
            text="Selected Move Pose (xyz / rpy)",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
            padx=8,
            pady=6,
        )
        pose_editor.grid(row=3, column=0, columnspan=5, sticky="ew", padx=(2, 2), pady=(6, 2))
        for i in range(12):
            pose_editor.columnconfigure(i, weight=0)
        pose_editor.columnconfigure(13, weight=1)

        for i, axis in enumerate(self.COORD_ORDER):
            tk.Label(
                pose_editor,
                text=f"{axis}:",
                font=("Times New Roman", 12),
                bg=self.BG,
                fg="#4D4D4D",
            ).grid(row=0, column=i * 2, sticky="e", padx=(2, 3), pady=2)
            entry = tk.Entry(
                pose_editor,
                textvariable=self.sequence_pose_vars[axis],
                bg=self.VAL_BG,
                fg=self.VAL_FG,
                width=8,
                justify="center",
                font=("Arial", 10),
                relief="flat",
                bd=0,
                highlightthickness=0,
                insertbackground=self.VAL_FG,
                state="disabled",
            )
            entry.grid(row=0, column=i * 2 + 1, sticky="w", padx=(0, 8), ipady=2, pady=2)
            entry.bind("<Return>", lambda _event: self._apply_selected_sequence_pose())
            self.sequence_pose_entries[axis] = entry

        self.sequence_pose_apply_btn = tk.Button(
            pose_editor,
            text="Apply",
            command=self._apply_selected_sequence_pose,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
            state="disabled",
        )
        self.sequence_pose_apply_btn.grid(row=0, column=12, sticky="w", padx=(0, 6))

        self.sequence_pose_reset_btn = tk.Button(
            pose_editor,
            text="Reset",
            command=self._reset_selected_sequence_pose,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
            state="disabled",
        )
        self.sequence_pose_reset_btn.grid(row=0, column=13, sticky="w")

        settings_box = tk.LabelFrame(
            parent,
            text="Recording Settings",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
            padx=8,
            pady=6,
        )
        settings_box.grid(row=2, column=0, sticky="ew", padx=(6, 6), pady=(2, 6))
        settings_box.columnconfigure(0, weight=0)
        settings_box.columnconfigure(1, weight=1)
        settings_box.columnconfigure(2, weight=0)
        settings_box.columnconfigure(3, weight=0)
        settings_box.columnconfigure(4, weight=0)

        tk.Label(
            settings_box,
            text="CSV folder:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=0, column=0, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            settings_box,
            textvariable=self.record_csv_dir_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            justify="left",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=0, column=1, columnspan=3, sticky="ew", padx=(0, 8), ipady=2, pady=2)
        tk.Button(
            settings_box,
            text="Browse",
            command=self._choose_record_csv_dir,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=0, column=4, sticky="w", pady=2)

        tk.Label(
            settings_box,
            text="CSV name:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=1, column=0, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            settings_box,
            textvariable=self.record_csv_name_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            justify="left",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=1, column=1, sticky="ew", padx=(0, 8), ipady=2, pady=2)

        tk.Label(
            settings_box,
            text="Speed (1-100):",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=1, column=2, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            settings_box,
            textvariable=self.record_speed_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=1, column=3, sticky="w", ipady=2, pady=2)

        tk.Label(
            settings_box,
            text="Record from step #:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=2, column=0, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            settings_box,
            textvariable=self.record_start_step_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=2, column=1, sticky="w", ipady=2, pady=2)

        tk.Label(
            settings_box,
            text="Record until step #:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=2, column=2, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            settings_box,
            textvariable=self.record_end_step_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            justify="center",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=2, column=3, sticky="w", ipady=2, pady=2)

        tk.Label(
            settings_box,
            text="Sequence file:",
            font=("Times New Roman", 13),
            bg=self.BG,
            fg="#4D4D4D",
        ).grid(row=3, column=0, sticky="e", padx=(2, 4), pady=2)
        tk.Entry(
            settings_box,
            textvariable=self.record_sequence_file_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            justify="left",
            font=("Arial", 10),
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        ).grid(row=3, column=1, columnspan=3, sticky="ew", padx=(0, 8), ipady=2, pady=2)
        tk.Button(
            settings_box,
            text="Browse",
            command=self._choose_sequence_file,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=10,
            pady=3,
            font=("Arial", 10),
        ).grid(row=3, column=4, sticky="w", pady=2)

        controls_box = tk.LabelFrame(
            parent,
            text="Run Controls",
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
            padx=8,
            pady=6,
        )
        controls_box.grid(row=3, column=0, sticky="ew", padx=(6, 6), pady=(2, 6))
        controls_box.columnconfigure(0, weight=1)
        controls_box.columnconfigure(1, weight=1)
        controls_box.columnconfigure(2, weight=1)

        tk.Button(
            controls_box,
            text="Save Sequence",
            command=self._save_sequence_file,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=(0, 6))

        tk.Button(
            controls_box,
            text="Load Sequence",
            command=self._load_sequence_file,
            bg=self.BTN_BG,
            fg="#666D75",
            activebackground=self.BTN_BG,
            activeforeground="#666D75",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=1, sticky="ew", padx=6, pady=(0, 6))

        tk.Button(
            controls_box,
            text="Abort Motion",
            command=self._abort_motion,
            bg="#F4DDDD",
            fg="#9A4040",
            activebackground="#F4DDDD",
            activeforeground="#9A4040",
            relief="groove",
            bd=1,
            padx=12,
            pady=3,
            font=("Arial", 11),
        ).grid(row=0, column=2, sticky="ew", padx=(6, 0), pady=(0, 6))

        tk.Button(
            controls_box,
            text="Run Sequence + Record CSV",
            command=self._run_sequence_and_record,
            bg=self.READ_BG,
            fg=self.READ_FG,
            activebackground=self.READ_BG,
            activeforeground=self.READ_FG,
            relief="groove",
            bd=1,
            padx=14,
            pady=3,
            font=("Arial", 11),
        ).grid(row=1, column=0, columnspan=3, sticky="ew")

        info_label = tk.Label(
            parent,
            text="Recording saves one robot snapshot when each selected sequence step is reached.",
            font=("Arial", 9),
            bg=self.BG,
            fg="#5D646D",
            justify="left",
            anchor="w",
        )
        info_label.grid(row=4, column=0, sticky="ew", padx=(6, 6), pady=(2, 0))

        status_label = tk.Label(
            parent,
            textvariable=self.status_var,
            bg=self.BG,
            fg="#5D646D",
            font=("Arial", 9),
            justify="left",
            anchor="w",
        )
        status_label.grid(row=5, column=0, sticky="ew", padx=(6, 6), pady=(8, 0))

        def update_wraplength(_: tk.Event | None = None) -> None:
            wrap = max(500, int(parent.winfo_width()) - 20)
            info_label.configure(wraplength=wrap)
            status_label.configure(wraplength=wrap)

        parent.bind("<Configure>", update_wraplength, add="+")
        self.after(0, update_wraplength)

        self._refresh_record_named_pose_options()
        self._refresh_sequence_list()

    def _build_value_control(
        self,
        parent: tk.Widget,
        label: str,
        value_var: tk.DoubleVar,
        on_minus: Callable[[], None],
        on_plus: Callable[[], None],
        on_set: Callable[[float], None],
        *,
        row: int,
        col: int,
    ) -> None:
        tk.Label(
            parent,
            text=label,
            font=("Times New Roman", 14),
            bg=self.BG,
            fg="#4D4D4D",
            width=2,
            anchor="e",
        ).grid(row=row, column=col, sticky="e", padx=(10, 6), pady=2)

        box = tk.Frame(parent, bg="#C9CED5", highlightbackground="#B8BEC7", highlightthickness=1)
        box.grid(row=row, column=col + 1, sticky="w", padx=(0, 18), pady=2)

        tk.Button(
            box,
            text="-",
            command=on_minus,
            bg=self.BTN_BG,
            fg="#7B8188",
            activebackground=self.BTN_BG,
            activeforeground="#7B8188",
            relief="flat",
            width=3,
            font=("Arial", 12),
            padx=0,
            pady=2,
        ).pack(side="left")

        entry_var = tk.StringVar(value=self._format_value(value_var.get()))
        val = tk.Entry(
            box,
            textvariable=entry_var,
            bg=self.VAL_BG,
            fg=self.VAL_FG,
            width=8,
            font=("Arial", 11),
            justify="center",
            relief="flat",
            bd=0,
            highlightthickness=0,
            insertbackground=self.VAL_FG,
        )
        val.pack(side="left")

        def refresh_label(*_: object) -> None:
            if self.focus_get() is val:
                return
            entry_var.set(self._format_value(value_var.get()))

        def commit_edit(_: tk.Event | None = None) -> None:
            raw = entry_var.get().strip().replace(",", ".")
            try:
                target = float(raw)
            except ValueError:
                entry_var.set(self._format_value(value_var.get()))
                self.status_var.set(f"Invalid number for {label}")
                return
            target = round(target, 2)
            current = round(float(value_var.get()), 2)
            if target == current:
                entry_var.set(self._format_value(current))
                return
            on_set(target)

        value_var.trace_add("write", refresh_label)
        val.bind("<Return>", commit_edit)
        val.bind("<KP_Enter>", commit_edit)
        val.bind("<FocusOut>", commit_edit)

        tk.Button(
            box,
            text="+",
            command=on_plus,
            bg=self.BTN_BG,
            fg="#7B8188",
            activebackground=self.BTN_BG,
            activeforeground="#7B8188",
            relief="flat",
            width=3,
            font=("Arial", 12),
            padx=0,
            pady=2,
        ).pack(side="left")

    def _get_home_pose(self) -> list[float]:
        return [self.HOME_X, self.HOME_Y, self.HOME_Z, self.HOME_RX, self.HOME_RY, self.HOME_RZ]

    def _set_home_pose(self, pose: list[float]) -> None:
        clamped, _ = self._clamp_coords([float(v) for v in pose])
        self.HOME_X = clamped[0]
        self.HOME_Y = clamped[1]
        self.HOME_Z = clamped[2]
        self.HOME_RX = clamped[3]
        self.HOME_RY = clamped[4]
        self.HOME_RZ = clamped[5]

    def _load_settings_file(self) -> None:
        if not self.settings_path.exists():
            try:
                self._save_settings_file()
            except Exception as exc:
                print(f"[quick_move_gui] WARN: could not create settings file: {exc}", flush=True)
            return

        try:
            raw = self.settings_path.read_text(encoding="utf-8")
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError("settings root must be an object")
        except Exception as exc:
            print(f"[quick_move_gui] WARN: failed to read settings file: {exc}", flush=True)
            return

        serial_data = data.get("serial", {})
        if isinstance(serial_data, dict):
            port = str(serial_data.get("port", self.serial_port)).strip()
            if port:
                self.serial_port = port
            baud_raw = serial_data.get("baudrate", self.serial_baud)
            try:
                baud = int(baud_raw)
            except Exception:
                baud = self.serial_baud
            if baud > 0:
                self.serial_baud = baud

        tool_length = self.tool_length_mm
        tool_data = data.get("tool", {})
        if isinstance(tool_data, dict):
            try:
                tool_length = float(tool_data.get("length_mm", tool_length))
            except Exception:
                tool_length = self.tool_length_mm
        elif "tool_length_mm" in data:
            try:
                tool_length = float(data.get("tool_length_mm", tool_length))
            except Exception:
                tool_length = self.tool_length_mm
        self.tool_length_mm = round(float(tool_length), 2)

        tol_data = data.get("tolerances", {})
        if isinstance(tol_data, dict):
            try:
                v = float(tol_data.get("record_move_xyz_mm", self.RECORD_MOVE_ARRIVE_TOL_MM))
                if v > 0:
                    self.RECORD_MOVE_ARRIVE_TOL_MM = v
            except Exception:
                pass
            try:
                v = float(tol_data.get("record_move_rpy_deg", self.RECORD_MOVE_RPY_ARRIVE_TOL_DEG))
                if v > 0:
                    self.RECORD_MOVE_RPY_ARRIVE_TOL_DEG = v
            except Exception:
                pass
            try:
                v = float(tol_data.get("general_xyz_mm", self.XYZ_ARRIVE_TOL_MM))
                if v > 0:
                    self.XYZ_ARRIVE_TOL_MM = v
            except Exception:
                pass
            try:
                v = float(tol_data.get("general_rpy_deg", self.HOME_RPY_ARRIVE_TOL_DEG))
                if v > 0:
                    self.HOME_RPY_ARRIVE_TOL_DEG = v
            except Exception:
                pass

        pose_defaults = self._get_home_pose()
        pose_data = data.get("home_pose", {})
        pose: list[float] = []
        for idx, axis in enumerate(self.COORD_ORDER):
            value = pose_defaults[idx]
            if isinstance(pose_data, dict):
                try:
                    value = float(pose_data.get(axis, value))
                except Exception:
                    value = pose_defaults[idx]
            pose.append(value)
        self._set_home_pose(pose)

        named_data = data.get("named_positions", {})
        loaded_positions: dict[str, list[float]] = {}
        if isinstance(named_data, dict):
            for raw_name, raw_pose in named_data.items():
                name = str(raw_name).strip()
                if not name:
                    continue
                try:
                    if isinstance(raw_pose, dict):
                        pose_values = [float(raw_pose.get(axis, pose_defaults[idx])) for idx, axis in enumerate(self.COORD_ORDER)]
                    elif isinstance(raw_pose, (list, tuple)) and len(raw_pose) >= 6:
                        pose_values = [float(v) for v in raw_pose[:6]]
                    else:
                        continue
                except Exception:
                    continue
                pose_values, _ = self._clamp_coords(pose_values)
                loaded_positions[name] = pose_values
        self.named_positions = loaded_positions

    def _save_settings_file(self) -> None:
        named_payload: dict[str, dict[str, float]] = {}
        for name, pose in sorted(self.named_positions.items(), key=lambda item: item[0].lower()):
            clamped_pose, _ = self._clamp_coords([float(v) for v in pose[:6]])
            named_payload[name] = {
                axis: round(clamped_pose[idx], 2)
                for idx, axis in enumerate(self.COORD_ORDER)
            }

        payload = {
            "serial": {
                "port": self.serial_port,
                "baudrate": int(self.serial_baud),
            },
            "home_pose": {
                axis: round(value, 2) for axis, value in zip(self.COORD_ORDER, self._get_home_pose())
            },
            "tool": {
                "length_mm": round(float(self.tool_length_mm), 2),
            },
            "tolerances": {
                "record_move_xyz_mm": round(float(self.RECORD_MOVE_ARRIVE_TOL_MM), 3),
                "record_move_rpy_deg": round(float(self.RECORD_MOVE_RPY_ARRIVE_TOL_DEG), 3),
                "general_xyz_mm": round(float(self.XYZ_ARRIVE_TOL_MM), 3),
                "general_rpy_deg": round(float(self.HOME_RPY_ARRIVE_TOL_DEG), 3),
            },
            "named_positions": named_payload,
        }
        text = json.dumps(payload, indent=2) + "\n"
        self.settings_path.write_text(text, encoding="utf-8")

    def _load_home_pose_from_display(self) -> None:
        for axis in self.COORD_ORDER:
            self.settings_home_vars[axis].set(self._format_value(self.coord_values[axis].get()))
        self.status_var.set("Home pose fields loaded from current displayed coords")

    def _apply_and_save_settings(self) -> None:
        if self.busy_event.is_set():
            self.status_var.set("Robot busy - wait for current motion before applying settings.")
            return

        port = self.settings_port_var.get().strip()
        if not port:
            self.status_var.set("Serial port cannot be empty.")
            return

        baud_text = self.settings_baud_var.get().strip()
        try:
            baud = int(baud_text)
        except Exception:
            self.status_var.set(f"Invalid baudrate: {baud_text!r}")
            return
        if baud <= 0:
            self.status_var.set("Baudrate must be a positive integer.")
            return

        home_values: list[float] = []
        for axis in self.COORD_ORDER:
            try:
                value = self._parse_float_text(self.settings_home_vars[axis].get(), axis)
            except ValueError as exc:
                self.status_var.set(str(exc))
                return
            home_values.append(value)

        try:
            tool_length = self._parse_float_text(self.settings_tool_length_var.get(), "tool length")
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        try:
            record_xyz_tol = self._parse_float_text(self.settings_tol_record_xyz_var.get(), "record XYZ tolerance")
            record_rpy_tol = self._parse_float_text(self.settings_tol_record_rpy_var.get(), "record RPY tolerance")
            general_xyz_tol = self._parse_float_text(self.settings_tol_general_xyz_var.get(), "general XYZ tolerance")
            general_rpy_tol = self._parse_float_text(self.settings_tol_general_rpy_var.get(), "general RPY tolerance")
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        if record_xyz_tol <= 0 or record_rpy_tol <= 0 or general_xyz_tol <= 0 or general_rpy_tol <= 0:
            self.status_var.set("Tolerances must be positive numbers.")
            return

        home_values, clamped = self._clamp_coords(home_values)
        for axis, value in zip(self.COORD_ORDER, home_values):
            self.settings_home_vars[axis].set(self._format_value(value))
        self.settings_tool_length_var.set(self._format_value(tool_length))
        self.settings_tol_record_xyz_var.set(self._format_value(record_xyz_tol))
        self.settings_tol_record_rpy_var.set(self._format_value(record_rpy_tol))
        self.settings_tol_general_xyz_var.set(self._format_value(general_xyz_tol))
        self.settings_tol_general_rpy_var.set(self._format_value(general_rpy_tol))
        if clamped:
            self.status_var.set("Home pose was clamped to robot limits.")

        prev_port = self.serial_port
        prev_baud = self.serial_baud
        self.serial_port = port
        self.serial_baud = baud
        self._set_home_pose(home_values)
        self.tool_length_mm = float(tool_length)
        self.RECORD_MOVE_ARRIVE_TOL_MM = float(record_xyz_tol)
        self.RECORD_MOVE_RPY_ARRIVE_TOL_DEG = float(record_rpy_tol)
        self.XYZ_ARRIVE_TOL_MM = float(general_xyz_tol)
        self.HOME_RPY_ARRIVE_TOL_DEG = float(general_rpy_tol)

        self.settings_port_var.set(self.serial_port)
        self.settings_baud_var.set(str(self.serial_baud))

        try:
            self._save_settings_file()
        except Exception as exc:
            self.status_var.set(f"Failed to save settings file: {exc}")
            return

        if self.mc is not None and (prev_port != self.serial_port or prev_baud != self.serial_baud):
            try:
                self.mc.close()
            except Exception:
                pass
            self.mc = None
            self.status_var.set(
                f"Settings saved. Connection reset; next command will reconnect ({self.serial_port} @ {self.serial_baud})."
            )
            return

        if self.mc is not None:
            try:
                self._apply_tool_reference(self.mc)
            except Exception as exc:
                self.status_var.set(f"Settings saved, but failed to apply tool offset: {exc}")
                return

        self.status_var.set(f"Settings saved to {self.settings_path}")

    def _read_target_pose_inputs(self) -> tuple[list[float], bool]:
        pose = [self._parse_float_text(self.target_pose_vars[axis].get(), axis) for axis in self.COORD_ORDER]
        pose, clamped = self._clamp_coords(pose)
        for axis, value in zip(self.COORD_ORDER, pose):
            self.target_pose_vars[axis].set(self._format_value(value))
        return pose, clamped

    def _refresh_named_positions_list(self, select_name: str | None = None) -> None:
        self._refresh_record_named_pose_options(select_name=select_name)
        if self.named_positions_listbox is None:
            self._refresh_sequence_list()
            return
        names = sorted(self.named_positions.keys(), key=str.lower)
        self.named_positions_listbox.delete(0, tk.END)
        for name in names:
            self.named_positions_listbox.insert(tk.END, name)
        if not names:
            self._refresh_sequence_list()
            return
        pick = select_name if select_name in names else names[0]
        idx = names.index(pick)
        self.named_positions_listbox.selection_clear(0, tk.END)
        self.named_positions_listbox.selection_set(idx)
        self.named_positions_listbox.activate(idx)
        self.named_positions_listbox.see(idx)
        self._refresh_sequence_list()

    def _get_selected_named_position_name(self) -> str | None:
        if self.named_positions_listbox is None:
            return None
        sel = self.named_positions_listbox.curselection()
        if not sel:
            return None
        return str(self.named_positions_listbox.get(sel[0]))

    def _store_named_position(self, name: str, pose: list[float]) -> tuple[bool, bool]:
        clean_name = str(name).strip()
        if not clean_name:
            raise ValueError("Enter a name for the position.")
        if len(pose) < 6:
            raise ValueError(f"Pose must have 6 values, got {pose}")

        clamped_pose, clamped = self._clamp_coords([float(v) for v in pose[:6]])
        existed = clean_name in self.named_positions
        self.named_positions[clean_name] = clamped_pose
        self._refresh_named_positions_list(select_name=clean_name)
        self.position_name_var.set(clean_name)
        self._save_settings_file()
        return existed, clamped

    def _save_named_position(self) -> None:
        name = self.position_name_var.get().strip()
        try:
            pose, _ = self._read_target_pose_inputs()
            existed, clamped = self._store_named_position(name, pose)
        except ValueError as exc:
            self.status_var.set(str(exc))
            return
        except Exception as exc:
            self.status_var.set(f"Failed to save named positions: {exc}")
            return
        display_name = self.position_name_var.get().strip() or name
        action = "Updated" if existed else "Saved"
        if clamped:
            self.status_var.set(f"{action} named pose '{display_name}' (pose clamped to limits).")
        else:
            self.status_var.set(f"{action} named pose '{display_name}'.")

    def _load_selected_named_position(self, _: tk.Event | None = None) -> None:
        name = self._get_selected_named_position_name()
        if not name:
            self.status_var.set("Select a named pose from the list.")
            return
        pose = self.named_positions.get(name)
        if not pose or len(pose) < 6:
            self.status_var.set(f"Named pose '{name}' is invalid.")
            return
        for axis, value in zip(self.COORD_ORDER, pose):
            self.target_pose_vars[axis].set(self._format_value(float(value)))
        self.position_name_var.set(name)
        self.status_var.set(f"Loaded named pose '{name}' into target fields.")

    def _delete_selected_named_position(self) -> None:
        name = self._get_selected_named_position_name()
        if not name:
            self.status_var.set("Select a named pose to delete.")
            return
        self.named_positions.pop(name, None)
        self._refresh_named_positions_list()
        try:
            self._save_settings_file()
        except Exception as exc:
            self.status_var.set(f"Deleted '{name}' in memory, but failed to save file: {exc}")
            return
        self.status_var.set(f"Deleted named pose '{name}'.")

    def _move_selected_named_position(self) -> None:
        name = self._get_selected_named_position_name()
        if not name:
            self.status_var.set("Select a named pose to move.")
            return
        pose = self.named_positions.get(name)
        if not pose or len(pose) < 6:
            self.status_var.set(f"Named pose '{name}' is invalid.")
            return
        for axis, value in zip(self.COORD_ORDER, pose):
            self.target_pose_vars[axis].set(self._format_value(float(value)))
        self.position_name_var.set(name)
        self._run_robot_task(
            f"Moving to named pose '{name}'...",
            lambda: self._move_to_pose_impl(*[float(v) for v in pose[:6]]),
        )

    def _free_joints(self) -> None:
        self._run_robot_task("Releasing joints for hand guide...", self._free_joints_impl)

    def _free_joints_impl(self) -> None:
        mc = self._ensure_robot()
        try:
            mc.stop()
        except Exception:
            pass

        released = False
        try:
            mc.release_all_servos()
            released = True
        except Exception:
            pass
        if not released:
            try:
                mc.set_free_mode(1)
                released = True
            except Exception as exc:
                raise RuntimeError("Failed to release joints for hand guide.") from exc

        time.sleep(0.15)
        self._set_status("Joints released. Move robot by hand, then click Lock Joints.")

    def _lock_joints(self) -> None:
        self._run_robot_task("Locking joints...", self._lock_joints_impl)

    def _lock_joints_impl(self) -> None:
        mc = self._ensure_robot()
        try:
            mc.set_free_mode(0)
        except Exception:
            pass
        self._ensure_motion_ready(mc)
        time.sleep(0.1)

        try:
            latest_angles = self._to_six(mc.get_angles(), "get_angles")
            self._update_joint_vars(latest_angles)
        except Exception:
            pass
        latest_coords = self._to_six(mc.get_coords(), "get_coords")
        self._update_coord_vars(latest_coords)
        for axis, value in zip(self.COORD_ORDER, latest_coords):
            self.target_pose_vars[axis].set(self._format_value(value))
        self._set_status("Joints locked. Current pose loaded.")

    def _save_current_pose_as_named(self) -> None:
        name = self.position_name_var.get().strip()
        if not name:
            self.status_var.set("Enter a name before saving hand-guided pose.")
            return
        self._run_robot_task(
            f"Saving hand-guided pose '{name}'...",
            lambda: self._save_current_pose_as_named_impl(name),
        )

    def _save_current_pose_as_named_impl(self, name: str) -> None:
        mc = self._ensure_robot()
        self._ensure_motion_ready(mc)
        time.sleep(0.1)

        coords = self._to_six(mc.get_coords(), "get_coords")
        coords, clamped = self._clamp_coords(coords)
        self._update_coord_vars(coords)
        for axis, value in zip(self.COORD_ORDER, coords):
            self.target_pose_vars[axis].set(self._format_value(value))
        try:
            latest_angles = self._to_six(mc.get_angles(), "get_angles")
            self._update_joint_vars(latest_angles)
        except Exception:
            pass

        try:
            existed, clamped_store = self._store_named_position(name, coords)
        except Exception as exc:
            raise RuntimeError(f"Failed to save hand-guided pose '{name}': {exc}") from exc

        action = "Updated" if existed else "Saved"
        if clamped or clamped_store:
            self._set_status(f"{action} hand-guided pose '{name}' (pose clamped to limits).")
        else:
            self._set_status(f"{action} hand-guided pose '{name}'.")

    def _refresh_record_named_pose_options(self, select_name: str | None = None) -> None:
        names = sorted(self.named_positions.keys(), key=str.lower)
        if self.record_named_pose_combo is not None:
            self.record_named_pose_combo.configure(values=names)
        if not names:
            self.record_named_pose_var.set("")
            return
        current = self.record_named_pose_var.get().strip()
        if select_name and select_name in names:
            self.record_named_pose_var.set(select_name)
        elif current in names:
            self.record_named_pose_var.set(current)
        else:
            self.record_named_pose_var.set(names[0])

    def _choose_record_csv_dir(self) -> None:
        initial = self.record_csv_dir_var.get().strip() or str(self.settings_path.parent)
        chosen = filedialog.askdirectory(initialdir=initial or None)
        if chosen:
            self.record_csv_dir_var.set(chosen)

    @staticmethod
    def _parse_pose_values(raw: Any) -> list[float] | None:
        if not isinstance(raw, (list, tuple)) or len(raw) < 6:
            return None
        out: list[float] = []
        try:
            for value in raw[:6]:
                out.append(float(value))
        except Exception:
            return None
        return out

    def _get_sequence_step_move_pose(
        self,
        step: dict[str, Any],
        *,
        strict: bool,
        step_index: int | None = None,
    ) -> list[float] | None:
        kind = str(step.get("kind", ""))
        base_pose: list[float] | None = None
        prefix = f"Step {step_index}: " if step_index is not None else ""

        if kind == "move_named":
            name = str(step.get("name", "")).strip()
            if not name:
                if strict:
                    raise ValueError(f"{prefix}named move has empty name.")
                return None
            pose = self.named_positions.get(name)
            if not pose or len(pose) < 6:
                if strict:
                    raise ValueError(f"{prefix}named pose '{name}' is missing.")
                return None
            base_pose = [float(v) for v in pose[:6]]
        elif kind == "home":
            base_pose = [float(v) for v in self._get_home_pose()]
        else:
            return None

        override_pose = self._parse_pose_values(step.get("pose_override"))
        resolved = override_pose if override_pose is not None else base_pose
        return self._clamp_coords(resolved)[0]

    def _format_pose_inline(self, pose: list[float]) -> str:
        values = [self._format_value(float(v)) for v in pose[:6]]
        return (
            f"x={values[0]}, y={values[1]}, z={values[2]}, "
            f"rx={values[3]}, ry={values[4]}, rz={values[5]}"
        )

    def _set_sequence_pose_editor_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for entry in self.sequence_pose_entries.values():
            entry.configure(state=state)
        if self.sequence_pose_apply_btn is not None:
            self.sequence_pose_apply_btn.configure(state=state)
        if self.sequence_pose_reset_btn is not None:
            self.sequence_pose_reset_btn.configure(state=state)

    def _sync_sequence_pose_editor_from_selection(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None or idx < 0 or idx >= len(self.sequence_steps):
            for axis in self.COORD_ORDER:
                self.sequence_pose_vars[axis].set("")
            self._set_sequence_pose_editor_enabled(False)
            return

        step = self.sequence_steps[idx]
        pose = self._get_sequence_step_move_pose(step, strict=False)
        if pose is None:
            for axis in self.COORD_ORDER:
                self.sequence_pose_vars[axis].set("")
            self._set_sequence_pose_editor_enabled(False)
            return

        for axis, value in zip(self.COORD_ORDER, pose):
            self.sequence_pose_vars[axis].set(self._format_value(value))
        self._set_sequence_pose_editor_enabled(True)

    def _apply_selected_sequence_pose(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None:
            self.status_var.set("Select a sequence step first.")
            return
        step = self.sequence_steps[idx]
        kind = str(step.get("kind", ""))
        if kind not in ("move_named", "home"):
            self.status_var.set("Pose editing is available only for MOVE steps.")
            return

        try:
            pose = [self._parse_float_text(self.sequence_pose_vars[axis].get(), axis) for axis in self.COORD_ORDER]
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        pose, clamped = self._clamp_coords(pose)
        step["pose_override"] = pose
        self._refresh_sequence_list(select_idx=idx)
        suffix = " (clamped to limits)." if clamped else "."
        self.status_var.set(f"Updated MOVE step {idx + 1} pose{suffix}")

    def _reset_selected_sequence_pose(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None:
            self.status_var.set("Select a sequence step first.")
            return
        step = self.sequence_steps[idx]
        kind = str(step.get("kind", ""))
        if kind not in ("move_named", "home"):
            self.status_var.set("Pose reset is available only for MOVE steps.")
            return

        had_override = "pose_override" in step
        step.pop("pose_override", None)
        self._refresh_sequence_list(select_idx=idx)
        if had_override:
            self.status_var.set(f"Reset MOVE step {idx + 1} to source pose.")
        else:
            self.status_var.set("Selected MOVE step already uses source pose.")

    def _describe_sequence_step(self, step: dict[str, Any], idx: int | None = None) -> str:
        kind = str(step.get("kind", "unknown"))
        prefix = f"{idx:02d}. " if idx is not None else ""
        if kind == "move_named":
            name = str(step.get("name", "?"))
            pose = self._get_sequence_step_move_pose(step, strict=False)
            custom_tag = " [edited]" if self._parse_pose_values(step.get("pose_override")) is not None else ""
            if pose is None:
                return f"{prefix}MOVE named '{name}'{custom_tag} (pose missing)"
            return f"{prefix}MOVE named '{name}'{custom_tag} | {self._format_pose_inline(pose)}"
        if kind == "home":
            pose = self._get_sequence_step_move_pose(step, strict=False)
            custom_tag = " [edited]" if self._parse_pose_values(step.get("pose_override")) is not None else ""
            if pose is None:
                return f"{prefix}MOVE home{custom_tag}"
            return f"{prefix}MOVE home{custom_tag} | {self._format_pose_inline(pose)}"
        if kind == "wait":
            try:
                wait_ms = int(step.get("ms", step.get("duration_ms", 0)))
            except Exception:
                wait_ms = 0
            return f"{prefix}WAIT {max(0, wait_ms)} ms"
        if kind == "vacuum":
            return f"{prefix}VACUUM {'ON' if bool(step.get('enabled')) else 'OFF'}"
        return f"{prefix}{kind}"

    def _normalize_record_step_range(self, total_steps: int, *, strict: bool) -> tuple[int, int]:
        max_step = max(1, int(total_steps))

        raw_start = self.record_start_step_var.get().strip()
        try:
            start_step = int(raw_start)
        except Exception as exc:
            if strict:
                raise ValueError("Invalid start step.") from exc
            start_step = 1
        start_step = max(1, min(max_step, start_step))

        raw_end = self.record_end_step_var.get().strip()
        if raw_end == "" or raw_end.lower() in ("end", "last"):
            end_step = max_step
        else:
            try:
                end_step = int(raw_end)
            except Exception as exc:
                if strict:
                    raise ValueError("Invalid end step.") from exc
                end_step = max_step
        end_step = max(start_step, min(max_step, end_step))

        self.record_start_step_var.set(str(start_step))
        self.record_end_step_var.set(str(end_step))
        return start_step, end_step

    def _refresh_sequence_list(self, select_idx: int | None = None) -> None:
        self._normalize_record_step_range(len(self.sequence_steps), strict=False)

        if self.sequence_listbox is None:
            return
        self.sequence_listbox.delete(0, tk.END)
        for i, step in enumerate(self.sequence_steps, start=1):
            self.sequence_listbox.insert(tk.END, self._describe_sequence_step(step, i))
        if not self.sequence_steps:
            self._sync_sequence_pose_editor_from_selection()
            return

        idx = 0 if select_idx is None else max(0, min(len(self.sequence_steps) - 1, int(select_idx)))
        self.sequence_listbox.selection_clear(0, tk.END)
        self.sequence_listbox.selection_set(idx)
        self.sequence_listbox.activate(idx)
        self.sequence_listbox.see(idx)
        self._sync_sequence_pose_editor_from_selection()

    def _get_selected_sequence_index(self) -> int | None:
        if self.sequence_listbox is None:
            return None
        sel = self.sequence_listbox.curselection()
        if not sel:
            return None
        return int(sel[0])

    def _copy_sequence_selected(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None:
            self.status_var.set("Select a sequence step to copy.")
            return
        self.sequence_step_clipboard = deepcopy(self.sequence_steps[idx])
        self.status_var.set(f"Copied step {idx + 1}.")

    def _paste_sequence_step(self) -> None:
        if self.sequence_step_clipboard is None:
            self.status_var.set("Clipboard empty. Copy a sequence step first.")
            return
        idx = self._get_selected_sequence_index()
        new_step = deepcopy(self.sequence_step_clipboard)
        if idx is None:
            self.sequence_steps.append(new_step)
            new_idx = len(self.sequence_steps) - 1
        else:
            new_idx = idx + 1
            self.sequence_steps.insert(new_idx, new_step)
        self._refresh_sequence_list(select_idx=new_idx)
        self.status_var.set(f"Pasted step at position {new_idx + 1}.")

    def _on_sequence_copy_shortcut(self, _: tk.Event | None = None) -> str:
        self._copy_sequence_selected()
        return "break"

    def _on_sequence_paste_shortcut(self, _: tk.Event | None = None) -> str:
        self._paste_sequence_step()
        return "break"

    def _add_sequence_move_named(self) -> None:
        name = self.record_named_pose_var.get().strip()
        if not name:
            self.status_var.set("Choose a named pose to add.")
            return
        if name not in self.named_positions:
            self.status_var.set(f"Named pose '{name}' does not exist.")
            return
        self.sequence_steps.append({"kind": "move_named", "name": name})
        self._refresh_sequence_list(select_idx=len(self.sequence_steps) - 1)
        self.status_var.set(f"Added move to named pose '{name}'.")

    def _add_sequence_home(self) -> None:
        self.sequence_steps.append({"kind": "home"})
        self._refresh_sequence_list(select_idx=len(self.sequence_steps) - 1)
        self.status_var.set("Added home move step.")

    def _add_sequence_wait(self) -> None:
        raw = self.record_wait_ms_var.get().strip()
        try:
            wait_ms = int(raw)
        except Exception:
            self.status_var.set("Invalid wait time. Use integer milliseconds (e.g., 500).")
            return
        if wait_ms < 0:
            self.status_var.set("Wait time must be >= 0 ms.")
            return
        self.record_wait_ms_var.set(str(wait_ms))
        self.sequence_steps.append({"kind": "wait", "ms": wait_ms})
        self._refresh_sequence_list(select_idx=len(self.sequence_steps) - 1)
        self.status_var.set(f"Added wait step ({wait_ms} ms).")

    def _add_sequence_vacuum(self, enabled: bool) -> None:
        self.sequence_steps.append({"kind": "vacuum", "enabled": bool(enabled)})
        self._refresh_sequence_list(select_idx=len(self.sequence_steps) - 1)
        self.status_var.set(f"Added vacuum {'ON' if enabled else 'OFF'} step.")

    def _move_sequence_up(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None:
            self.status_var.set("Select a sequence step first.")
            return
        if idx <= 0:
            return
        self.sequence_steps[idx - 1], self.sequence_steps[idx] = self.sequence_steps[idx], self.sequence_steps[idx - 1]
        self._refresh_sequence_list(select_idx=idx - 1)

    def _move_sequence_down(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None:
            self.status_var.set("Select a sequence step first.")
            return
        if idx >= len(self.sequence_steps) - 1:
            return
        self.sequence_steps[idx + 1], self.sequence_steps[idx] = self.sequence_steps[idx], self.sequence_steps[idx + 1]
        self._refresh_sequence_list(select_idx=idx + 1)

    def _delete_sequence_selected(self) -> None:
        idx = self._get_selected_sequence_index()
        if idx is None:
            self.status_var.set("Select a sequence step to delete.")
            return
        removed = self.sequence_steps.pop(idx)
        next_idx = min(idx, len(self.sequence_steps) - 1)
        self._refresh_sequence_list(select_idx=next_idx if next_idx >= 0 else None)
        self.status_var.set(f"Deleted step: {self._describe_sequence_step(removed)}")

    def _clear_sequence(self) -> None:
        self.sequence_steps.clear()
        self._refresh_sequence_list()
        self.status_var.set("Sequence cleared.")

    def _build_record_csv_path(self) -> Path:
        folder_text = self.record_csv_dir_var.get().strip()
        folder = Path(folder_text) if folder_text else self.settings_path.parent
        name_text = self.record_csv_name_var.get().strip() or "motion_record.csv"
        name_path = Path(name_text)
        stem = name_path.stem.strip() or "motion_record"
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        candidate = folder / f"{stem}_{stamp}.csv"
        suffix_idx = 1
        while candidate.exists():
            candidate = folder / f"{stem}_{stamp}_{suffix_idx}.csv"
            suffix_idx += 1
        return candidate

    def _resolve_sequence_steps(self) -> list[dict[str, Any]]:
        sequence_snapshot = [dict(step) for step in self.sequence_steps]
        resolved_steps: list[dict[str, Any]] = []
        for i, step in enumerate(sequence_snapshot, start=1):
            kind = str(step.get("kind", ""))
            if kind == "move_named":
                name = str(step.get("name", "")).strip()
                pose = self._get_sequence_step_move_pose(step, strict=True, step_index=i)
                label = f"named:{name}"
                if self._parse_pose_values(step.get("pose_override")) is not None:
                    label += ":edited"
                resolved_steps.append({"kind": "move", "label": label, "pose": pose})
            elif kind == "home":
                pose = self._get_sequence_step_move_pose(step, strict=True, step_index=i)
                label = "home"
                if self._parse_pose_values(step.get("pose_override")) is not None:
                    label += ":edited"
                resolved_steps.append({"kind": "move", "label": label, "pose": pose})
            elif kind == "wait":
                try:
                    wait_ms = int(step.get("ms", step.get("duration_ms", 0)))
                except Exception as exc:
                    raise ValueError(f"Step {i}: invalid wait value.") from exc
                if wait_ms < 0:
                    raise ValueError(f"Step {i}: wait value must be >= 0 ms.")
                resolved_steps.append({"kind": "wait", "ms": wait_ms})
            elif kind == "vacuum":
                resolved_steps.append({"kind": "vacuum", "enabled": bool(step.get("enabled"))})
            else:
                raise ValueError(f"Step {i}: unsupported step kind '{kind}'.")
        return resolved_steps

    def _build_sequence_file_path(self) -> Path:
        raw = self.record_sequence_file_var.get().strip()
        if not raw:
            raw = str(self.settings_path.parent / "motion_sequence.json")
        path = Path(raw).expanduser()
        if path.suffix.lower() != ".json":
            path = path.with_suffix(".json")
        self.record_sequence_file_var.set(str(path))
        return path

    def _choose_sequence_file(self) -> None:
        initial = self._build_sequence_file_path()
        chosen = filedialog.askopenfilename(
            initialdir=str(initial.parent),
            initialfile=initial.name,
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if chosen:
            self.record_sequence_file_var.set(chosen)

    def _save_sequence_file(self) -> None:
        if self.busy_event.is_set():
            self.status_var.set("Robot busy - wait for current motion before saving sequence.")
            return
        path = self._build_sequence_file_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "sequence_steps": self.sequence_steps,
            "defaults": {
                "speed": self.record_speed_var.get().strip(),
                "start_step": self.record_start_step_var.get().strip(),
                "end_step": self.record_end_step_var.get().strip(),
                "wait_ms": self.record_wait_ms_var.get().strip(),
            },
        }
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self.status_var.set(f"Sequence saved to {path}")

    def _load_sequence_file(self) -> None:
        if self.busy_event.is_set():
            self.status_var.set("Robot busy - wait for current motion before loading sequence.")
            return
        path = self._build_sequence_file_path()
        if not path.exists():
            self.status_var.set(f"Sequence file not found: {path}")
            return

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.status_var.set(f"Failed to read sequence file: {exc}")
            return

        if isinstance(data, list):
            raw_steps = data
            defaults = {}
        elif isinstance(data, dict):
            raw_steps = data.get("sequence_steps", data.get("steps", []))
            defaults = data.get("defaults", {})
        else:
            self.status_var.set("Invalid sequence file format.")
            return

        if not isinstance(raw_steps, list):
            self.status_var.set("Invalid sequence file: 'sequence_steps' must be a list.")
            return

        loaded_steps: list[dict[str, Any]] = []
        missing_named = 0
        for idx, item in enumerate(raw_steps, start=1):
            if not isinstance(item, dict):
                self.status_var.set(f"Invalid step {idx}: expected object.")
                return
            kind = str(item.get("kind", "")).strip()
            if kind == "move_named":
                name = str(item.get("name", "")).strip()
                if not name:
                    self.status_var.set(f"Invalid step {idx}: move_named requires non-empty name.")
                    return
                if name not in self.named_positions:
                    missing_named += 1
                step_obj: dict[str, Any] = {"kind": "move_named", "name": name}
                override_pose = self._parse_pose_values(item.get("pose_override", item.get("pose")))
                if override_pose is not None:
                    step_obj["pose_override"] = self._clamp_coords(override_pose)[0]
                loaded_steps.append(step_obj)
            elif kind == "home":
                step_obj: dict[str, Any] = {"kind": "home"}
                override_pose = self._parse_pose_values(item.get("pose_override", item.get("pose")))
                if override_pose is not None:
                    step_obj["pose_override"] = self._clamp_coords(override_pose)[0]
                loaded_steps.append(step_obj)
            elif kind == "wait":
                try:
                    wait_ms = int(item.get("ms", item.get("duration_ms", 0)))
                except Exception:
                    self.status_var.set(f"Invalid step {idx}: wait value must be integer milliseconds.")
                    return
                if wait_ms < 0:
                    self.status_var.set(f"Invalid step {idx}: wait value must be >= 0 ms.")
                    return
                loaded_steps.append({"kind": "wait", "ms": wait_ms})
            elif kind == "vacuum":
                loaded_steps.append({"kind": "vacuum", "enabled": bool(item.get("enabled"))})
            else:
                self.status_var.set(f"Invalid step {idx}: unsupported kind '{kind}'.")
                return

        self.sequence_steps = loaded_steps
        self._refresh_sequence_list(select_idx=0 if loaded_steps else None)

        if isinstance(defaults, dict):
            if "speed" in defaults:
                self.record_speed_var.set(str(defaults["speed"]))
            if "start_step" in defaults:
                self.record_start_step_var.set(str(defaults["start_step"]))
            if "end_step" in defaults:
                self.record_end_step_var.set(str(defaults["end_step"]))
            if "wait_ms" in defaults:
                self.record_wait_ms_var.set(str(defaults["wait_ms"]))
            self._refresh_sequence_list(select_idx=0 if loaded_steps else None)

        if missing_named > 0:
            self.status_var.set(
                f"Loaded {len(loaded_steps)} sequence steps from {path} "
                f"({missing_named} named poses are missing in current settings)."
            )
        else:
            self.status_var.set(f"Loaded {len(loaded_steps)} sequence steps from {path}")

    def _run_sequence_and_record(self) -> None:
        if not self.sequence_steps:
            self.status_var.set("Sequence is empty. Add steps before recording.")
            return
        try:
            speed = int(self.record_speed_var.get().strip())
        except Exception:
            self.status_var.set("Invalid speed. Use integer 1-100.")
            return
        speed = max(1, min(100, speed))
        self.record_speed_var.set(str(speed))

        try:
            start_step, end_step = self._normalize_record_step_range(len(self.sequence_steps), strict=True)
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        output_path = self._build_record_csv_path()
        try:
            resolved_steps = self._resolve_sequence_steps()
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        self._run_robot_task(
            "Running sequence and recording CSV...",
            lambda: self._run_sequence_and_record_impl(resolved_steps, output_path, speed, start_step, end_step),
        )

    def _run_sequence_and_record_impl(
        self,
        steps: list[dict[str, Any]],
        output_path: Path,
        speed: int,
        start_step: int,
        end_step: int,
    ) -> None:
        mc = self._ensure_robot()
        self._ensure_motion_ready(mc)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        start_t = time.monotonic()
        vacuum_state = 0
        sample_count = 0

        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp_local",
                    "t_sec",
                    "step_index",
                    "step_action",
                    "vacuum_on",
                    "atomic_snapshot",
                    "j1",
                    "j2",
                    "j3",
                    "j4",
                    "j5",
                    "j6",
                    "x",
                    "y",
                    "z",
                    "rx",
                    "ry",
                    "rz",
                ]
            )

            def write_sample(step_idx: int, step_action: str, angles: list[float], coords: list[float], atomic: bool) -> None:
                nonlocal sample_count
                t_rel = time.monotonic() - start_t
                t_local = datetime.now().astimezone().isoformat(timespec="milliseconds")
                row = [
                    t_local,
                    round(t_rel, 4),
                    step_idx,
                    step_action,
                    vacuum_state,
                    1 if atomic else 0,
                    round(float(angles[0]), 3),
                    round(float(angles[1]), 3),
                    round(float(angles[2]), 3),
                    round(float(angles[3]), 3),
                    round(float(angles[4]), 3),
                    round(float(angles[5]), 3),
                    round(float(coords[0]), 3),
                    round(float(coords[1]), 3),
                    round(float(coords[2]), 3),
                    round(float(coords[3]), 3),
                    round(float(coords[4]), 3),
                    round(float(coords[5]), 3),
                ]
                writer.writerow(row)
                sample_count += 1

            total = len(steps)
            for idx, step in enumerate(steps, start=1):
                self._check_abort(mc)
                kind = str(step.get("kind", ""))
                should_record = start_step <= idx <= end_step
                if kind == "move":
                    pose = self._clamp_coords([float(v) for v in step["pose"][:6]])[0]
                    action = f"move:{step.get('label', '')}"
                    self._set_status(f"Sequence step {idx}/{total}: {action}")
                    current = self._to_six(mc.get_coords(), "get_coords")
                    try:
                        self._send_pose_target(
                            mc,
                            current,
                            pose,
                            f"Step {idx}/{total} {action}",
                            rpy_tolerance_deg=self.RECORD_MOVE_RPY_ARRIVE_TOL_DEG,
                            speed=speed,
                            sample_dt=self.RECORD_MOVE_POLL_S,
                            xyz_tolerance_mm=self.RECORD_MOVE_ARRIVE_TOL_MM,
                            use_sync=False,
                        )
                    except RuntimeError as exc:
                        if "incomplete:" not in str(exc):
                            raise
                        latest = self._to_six(mc.get_coords(), "get_coords")
                        xyz_err = self._xyz_distance(latest[:3], pose[:3])
                        rpy_err = max(
                            abs(float(latest[3]) - float(pose[3])),
                            abs(float(latest[4]) - float(pose[4])),
                            abs(float(latest[5]) - float(pose[5])),
                        )
                        if xyz_err > self.XYZ_ARRIVE_TOL_MM or rpy_err > self.HOME_RPY_ARRIVE_TOL_DEG:
                            raise RuntimeError(f"{exc} (post-check xyz={xyz_err:.1f} mm, rpy={rpy_err:.1f} deg)") from exc
                        self._set_status(f"Sequence step {idx}/{total}: {action} near target; continuing.")

                    if should_record:
                        angles, coords, atomic = self._read_angles_coords_snapshot(mc)
                        write_sample(idx, action, angles, coords, atomic)
                elif kind == "wait":
                    wait_ms = max(0, int(step.get("ms", 0)))
                    wait_s = wait_ms / 1000.0
                    self._set_status(f"Sequence step {idx}/{total}: wait {wait_ms} ms")
                    self._sleep_abortable(wait_s, mc)
                    if should_record:
                        angles, coords, atomic = self._read_angles_coords_snapshot(mc)
                        write_sample(idx, f"wait:{wait_ms}ms", angles, coords, atomic)
                elif kind == "vacuum":
                    enabled = bool(step.get("enabled"))
                    self._set_status(f"Sequence step {idx}/{total}: vacuum {'ON' if enabled else 'OFF'}")
                    self._set_vacuum_impl(enabled)
                    vacuum_state = 1 if enabled else 0
                    if should_record:
                        angles, coords, atomic = self._read_angles_coords_snapshot(mc)
                        write_sample(idx, f"vacuum:{'on' if enabled else 'off'}", angles, coords, atomic)
                else:
                    raise RuntimeError(f"Unsupported step kind: {kind}")

            csv_file.flush()

        self._set_status(
            f"Sequence complete. Saved {sample_count} samples to {output_path} "
            f"(steps {start_step}-{end_step}, speed={speed})."
        )

    def _abort_motion(self) -> None:
        self.abort_event.set()
        if self.mc is not None:
            try:
                self.mc.stop()
            except Exception:
                pass
        if self.busy_event.is_set():
            self.status_var.set("Abort requested. Stopping robot...")
        else:
            self.status_var.set("Abort command sent.")

    def _check_abort(self, mc: Any | None = None) -> None:
        if not self.abort_event.is_set():
            return
        if mc is not None:
            try:
                mc.stop()
            except Exception:
                pass
        raise RuntimeError("Motion aborted by user.")

    def _sleep_abortable(self, duration_s: float, mc: Any | None = None) -> None:
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while True:
            self._check_abort(mc)
            remain = deadline - time.monotonic()
            if remain <= 0:
                return
            time.sleep(min(0.02, remain))

    def _run_robot_task(self, label: str, action: Callable[[], None]) -> None:
        if self.busy_event.is_set():
            self.status_var.set("Robot busy - wait for current motion.")
            return

        self.abort_event.clear()
        self.busy_event.set()
        self.status_var.set(label)

        def worker() -> None:
            try:
                action()
            except Exception as exc:
                if str(exc) == "Motion aborted by user.":
                    self._set_status("Motion aborted.")
                    return
                msg = f"{label} failed: {exc}"
                self._set_status(msg)
                self._log_error(msg, exc)
            finally:
                self.busy_event.clear()

        threading.Thread(target=worker, daemon=True).start()

    def _ensure_robot(self) -> Any:
        if self.mc is not None:
            return self.mc

        if MyCobot280 is None:
            raise RuntimeError("pymycobot is not installed in this Python environment.")

        mc = MyCobot280(self.serial_port, self.serial_baud)
        try:
            mc.power_on()
        except Exception:
            pass

        self._configure_motion(mc)
        self._apply_tool_reference(mc)
        self.mc = mc
        self._set_status(f"Connected to {self.serial_port} @ {self.serial_baud}")
        return mc

    def _ensure_motion_ready(self, mc: Any) -> None:
        last_power = None
        last_servo = None
        for _ in range(3):
            self._check_abort(mc)
            try:
                mc.power_on()
            except Exception:
                pass
            try:
                if mc.is_paused() == 1:
                    mc.resume()
            except Exception:
                pass
            try:
                mc.focus_all_servos()
            except Exception:
                pass
            time.sleep(0.25)
            try:
                last_power = mc.is_power_on()
            except Exception:
                last_power = None
            try:
                last_servo = mc.is_all_servo_enable()
            except Exception:
                last_servo = None
            if last_power == 1 and last_servo == 1:
                time.sleep(self.MOTION_READY_SETTLE_S)
                return

        raise RuntimeError(
            "Robot not motion-ready "
            f"(is_power_on={last_power}, is_all_servo_enable={last_servo}). "
            "Check emergency stop, motor release state, and power."
        )

    @staticmethod
    def _configure_motion(mc: Any) -> None:
        try:
            mc.clear_queue()
        except Exception:
            pass
        for fn_name, arg in (
            ("set_fresh_mode", 0),
            ("set_reference_frame", 0),
            ("set_end_type", 1),
            ("set_movement_type", 1),
        ):
            try:
                getattr(mc, fn_name)(arg)
            except Exception:
                pass

    def _apply_tool_reference(self, mc: Any) -> None:
        tool_ref = [0.0, 0.0, float(self.tool_length_mm), 0.0, 0.0, 0.0]
        mc.set_tool_reference(tool_ref)
        mc.set_end_type(1)

    @staticmethod
    def _to_six(values: Any, label: str) -> list[float]:
        if not isinstance(values, (list, tuple)) or len(values) < 6:
            raise RuntimeError(f"{label}: robot returned invalid data: {values}")
        return [float(v) for v in values[:6]]

    def _set_status(self, text: str) -> None:
        self.after(0, lambda: self.status_var.set(text))

    def _update_joint_vars(self, angles: list[float]) -> None:
        def apply() -> None:
            for name, value in zip(self.JOINT_ORDER, angles):
                self.joint_values[name].set(round(float(value), 2))

        self.after(0, apply)

    def _update_coord_vars(self, coords: list[float]) -> None:
        def apply() -> None:
            for name, value in zip(self.COORD_ORDER, coords):
                self.coord_values[name].set(round(float(value), 2))

        self.after(0, apply)

    def _read_angles(self) -> None:
        self._run_robot_task("Reading angles...", self._read_angles_impl)

    def _read_angles_impl(self) -> None:
        mc = self._ensure_robot()
        angles = self._to_six(mc.get_angles(), "get_angles")
        self._update_joint_vars(angles)
        self._set_status("Angles updated")

    def _read_coords(self) -> None:
        self._run_robot_task("Reading coords...", self._read_coords_impl)

    def _read_coords_impl(self) -> None:
        mc = self._ensure_robot()
        coords = self._to_six(mc.get_coords(), "get_coords")
        self._update_coord_vars(coords)
        self._set_status("Coords updated")

    def _step_joint(self, joint_name: str, delta: float) -> None:
        self._run_robot_task(f"Moving {joint_name}...", lambda: self._step_joint_impl(joint_name, delta))

    def _step_joint_impl(self, joint_name: str, delta: float) -> None:
        mc = self._ensure_robot()
        idx = self.JOINT_ORDER.index(joint_name) + 1
        current = self._to_six(mc.get_angles(), "get_angles")
        target = round(current[idx - 1] + delta, 2)
        self._send_joint_target(mc, idx, target, current)
        self._set_status(f"{joint_name} -> {self._format_value(target)}")

    def _set_joint(self, joint_name: str, target: float) -> None:
        self._run_robot_task(f"Setting {joint_name}...", lambda: self._set_joint_impl(joint_name, target))

    def _set_joint_impl(self, joint_name: str, target: float) -> None:
        mc = self._ensure_robot()
        idx = self.JOINT_ORDER.index(joint_name) + 1
        current = self._to_six(mc.get_angles(), "get_angles")
        self._send_joint_target(mc, idx, round(target, 2), current)
        self._set_status(f"{joint_name} set to {self._format_value(target)}")

    def _send_joint_target(self, mc: Any, joint_idx: int, target: float, current_angles: list[float]) -> None:
        self._ensure_motion_ready(mc)
        self._check_abort(mc)
        try:
            mc.send_angle(joint_idx, target, self.SPEED)
        except Exception:
            current_angles[joint_idx - 1] = target
            mc.send_angles(current_angles, self.SPEED)

        time.sleep(0.08)
        try:
            latest = self._to_six(mc.get_angles(), "get_angles")
        except Exception:
            latest = current_angles
            latest[joint_idx - 1] = target

        self._update_joint_vars(latest)

    def _step_coord(self, coord_name: str, delta: float) -> None:
        self._run_robot_task(f"Moving {coord_name}...", lambda: self._step_coord_impl(coord_name, delta))

    def _step_coord_impl(self, coord_name: str, delta: float) -> None:
        mc = self._ensure_robot()
        idx = self.COORD_ORDER.index(coord_name) + 1
        current = self._to_six(mc.get_coords(), "get_coords")
        target = round(current[idx - 1] + delta, 2)
        self._send_coord_target(mc, idx, target, current)
        self._set_status(f"{coord_name} -> {self._format_value(target)}")

    def _set_coord(self, coord_name: str, target: float) -> None:
        self._run_robot_task(f"Setting {coord_name}...", lambda: self._set_coord_impl(coord_name, target))

    def _set_coord_impl(self, coord_name: str, target: float) -> None:
        mc = self._ensure_robot()
        idx = self.COORD_ORDER.index(coord_name) + 1
        current = self._to_six(mc.get_coords(), "get_coords")
        self._send_coord_target(mc, idx, round(target, 2), current)
        self._set_status(f"{coord_name} set to {self._format_value(target)}")

    def _send_coord_target(self, mc: Any, coord_idx: int, target: float, current_coords: list[float]) -> None:
        self._ensure_motion_ready(mc)
        self._check_abort(mc)
        try:
            mc.send_coord(coord_idx, target, self.SPEED)
        except Exception:
            current_coords[coord_idx - 1] = target
            mc.send_coords(current_coords, self.SPEED, self.LINEAR_MODE)

        time.sleep(0.08)
        try:
            latest = self._to_six(mc.get_coords(), "get_coords")
        except Exception:
            latest = current_coords
            latest[coord_idx - 1] = target

        self._update_coord_vars(latest)

    def _set_vacuum(self, enabled: bool) -> None:
        state = "ON" if enabled else "OFF"
        self._run_robot_task(f"Turning vacuum {state}...", lambda: self._set_vacuum_impl(enabled))

    def _set_vacuum_impl(self, enabled: bool) -> None:
        mc = self._ensure_robot()
        on_value = 1 if enabled else 0
        off_value = 0 if enabled else 1
        try:
            mc.set_basic_output(self.VAC_PIN_ON, on_value)
            mc.set_basic_output(self.VAC_PIN_OFF, off_value)
        except Exception:
            # Compatibility fallback for controllers exposing digital output API.
            mc.set_digital_output(self.VAC_PIN_ON, on_value)
            mc.set_digital_output(self.VAC_PIN_OFF, off_value)
        self._set_status(f"Vacuum {'ON' if enabled else 'OFF'}")

    def _load_target_pose_from_display(self) -> None:
        for axis in self.COORD_ORDER:
            self.target_pose_vars[axis].set(self._format_value(self.coord_values[axis].get()))
        self.status_var.set("Target pose loaded from current displayed coords")

    @staticmethod
    def _parse_float_text(raw: str, axis: str) -> float:
        text = raw.strip().replace(",", ".")
        try:
            return round(float(text), 2)
        except Exception as exc:
            raise ValueError(f"Invalid {axis} value: {raw!r}") from exc

    def _move_to_pose(self) -> None:
        try:
            pose, _ = self._read_target_pose_inputs()
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        x, y, z, rx, ry, rz = pose
        self._run_robot_task(
            "Moving to target pose...",
            lambda: self._move_to_pose_impl(x, y, z, rx, ry, rz),
        )

    def _go_zero_joints(self) -> None:
        self._run_robot_task("Moving to zero joints...", self._go_zero_joints_impl)

    def _go_zero_joints_impl(self) -> None:
        mc = self._ensure_robot()
        self._ensure_motion_ready(mc)
        start = self._to_six(mc.get_angles(), "get_angles")
        target = [0.0] * 6
        max_delta = max(abs(v) for v in start)
        step_deg = max(0.5, float(self.ZERO_JOINT_STEP_DEG))
        steps = max(1, int(ceil(max_delta / step_deg)))

        for i in range(1, steps + 1):
            self._check_abort(mc)
            ratio = i / steps
            waypoint = [round(s + (t - s) * ratio, 2) for s, t in zip(start, target)]
            self._set_status(f"Zero joints... step {i}/{steps}")
            mc.send_angles(waypoint, self.SPEED)
            time.sleep(0.05)

        time.sleep(0.15)
        latest_angles = self._to_six(mc.get_angles(), "get_angles")
        self._update_joint_vars(latest_angles)
        try:
            latest_coords = self._to_six(mc.get_coords(), "get_coords")
            self._update_coord_vars(latest_coords)
        except Exception:
            pass

        max_err = max(abs(v) for v in latest_angles)
        if max_err > self.ZERO_ARRIVE_TOL_DEG:
            raise RuntimeError(f"Zero position incomplete: max joint error {max_err:.1f} deg.")
        self._set_status("Reached zero joints position")

    def _go_home_position(self) -> None:
        self._run_robot_task("Moving to home position...", self._go_home_position_impl)

    def _go_home_position_impl(self) -> None:
        mc = self._ensure_robot()
        self._ensure_motion_ready(mc)
        current = self._to_six(mc.get_coords(), "get_coords")
        target = self._clamp_coords(
            [self.HOME_X, self.HOME_Y, self.HOME_Z, self.HOME_RX, self.HOME_RY, self.HOME_RZ]
        )[0]

        self.target_pose_vars["x"].set(self._format_value(self.HOME_X))
        self.target_pose_vars["y"].set(self._format_value(self.HOME_Y))
        self.target_pose_vars["z"].set(self._format_value(self.HOME_Z))
        self.target_pose_vars["rx"].set(self._format_value(self.HOME_RX))
        self.target_pose_vars["ry"].set(self._format_value(self.HOME_RY))
        self.target_pose_vars["rz"].set(self._format_value(self.HOME_RZ))

        latest_coords = self._send_pose_target(
            mc,
            current,
            target,
            "Home move",
            self.HOME_RPY_ARRIVE_TOL_DEG,
            use_sync=False,
        )

        self._update_coord_vars(latest_coords)
        try:
            latest_angles = self._to_six(mc.get_angles(), "get_angles")
            self._update_joint_vars(latest_angles)
        except Exception:
            pass

        xyz_err = self._xyz_distance(latest_coords[:3], target[:3])
        rpy_err = self._max_rpy_error(latest_coords, target)
        if xyz_err <= self.XYZ_ARRIVE_TOL_MM and rpy_err <= self.HOME_RPY_ARRIVE_TOL_DEG:
            self._set_status(
                f"Reached home position ({self._format_value(self.HOME_X)}, "
                f"{self._format_value(self.HOME_Y)}, {self._format_value(self.HOME_Z)}) "
                f"rpy=({self._format_value(self.HOME_RX)}, {self._format_value(self.HOME_RY)}, {self._format_value(self.HOME_RZ)})"
            )
        else:
            self._set_status(
                f"Moved near home (closest reachable): xyz err {xyz_err:.1f} mm, rpy err {rpy_err:.1f} deg."
            )

    def _move_to_pose_impl(self, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> None:
        mc = self._ensure_robot()
        self._ensure_motion_ready(mc)
        current = self._to_six(mc.get_coords(), "get_coords")
        target_coords = self._clamp_coords([x, y, z, rx, ry, rz])[0]
        latest_coords = self._send_pose_target(mc, current, target_coords, "Target pose move", use_sync=False)
        self._update_coord_vars(latest_coords)

        try:
            latest_angles = self._to_six(mc.get_angles(), "get_angles")
            self._update_joint_vars(latest_angles)
        except Exception:
            pass

        xyz_err = self._xyz_distance(latest_coords[:3], target_coords[:3])
        rpy_err = self._max_rpy_error(latest_coords, target_coords)
        if xyz_err <= self.XYZ_ARRIVE_TOL_MM and rpy_err <= self.HOME_RPY_ARRIVE_TOL_DEG:
            self._set_status(
                f"Moved to pose xyz=({self._format_value(target_coords[0])}, "
                f"{self._format_value(target_coords[1])}, {self._format_value(target_coords[2])}) "
                f"rpy=({self._format_value(target_coords[3])}, "
                f"{self._format_value(target_coords[4])}, {self._format_value(target_coords[5])})"
            )
        else:
            self._set_status(f"Moved near target (closest reachable): xyz err {xyz_err:.1f} mm, rpy err {rpy_err:.1f} deg.")

    def _read_angles_coords_snapshot(self, mc: Any) -> tuple[list[float], list[float], bool]:
        try:
            combined = mc.get_angles_coords()
            if isinstance(combined, (list, tuple)) and len(combined) >= 12:
                angles = [float(v) for v in combined[:6]]
                coords = [float(v) for v in combined[6:12]]
                return angles, coords, True
        except Exception:
            pass

        angles = self._to_six(mc.get_angles(), "get_angles")
        coords = self._to_six(mc.get_coords(), "get_coords")
        return angles, coords, False

    def _send_pose_target(
        self,
        mc: Any,
        current: list[float],
        target: list[float],
        label: str,
        rpy_tolerance_deg: float | None = None,
        *,
        speed: int | None = None,
        sample_callback: Callable[[list[float], list[float], bool], None] | None = None,
        sample_dt: float = 0.08,
        xyz_tolerance_mm: float | None = None,
        use_sync: bool = True,
        allow_closest: bool = True,
    ) -> list[float]:
        current = self._clamp_coords([float(v) for v in current[:6]])[0]
        target = self._clamp_coords([float(v) for v in target[:6]])[0]
        xyz_tol = self.XYZ_ARRIVE_TOL_MM if xyz_tolerance_mm is None else max(0.1, float(xyz_tolerance_mm))
        rpy_tol = self.HOME_RPY_ARRIVE_TOL_DEG if rpy_tolerance_deg is None else float(rpy_tolerance_deg)

        try:
            return self._send_pose_target_exact(
                mc,
                current,
                target,
                label,
                rpy_tolerance_deg=rpy_tolerance_deg,
                speed=speed,
                sample_callback=sample_callback,
                sample_dt=sample_dt,
                xyz_tolerance_mm=xyz_tolerance_mm,
                use_sync=use_sync,
            )
        except Exception as exc:
            if str(exc) == "Motion aborted by user.":
                raise
            latest = list(current)
            try:
                latest = self._to_six(mc.get_coords(), "get_coords")
            except Exception:
                pass

            latest_xyz_err = self._xyz_distance(latest[:3], target[:3])
            latest_rpy_err = self._max_rpy_error(latest, target)
            if latest_xyz_err <= xyz_tol and latest_rpy_err <= rpy_tol:
                return latest

            if (
                not allow_closest
                or not self.CLOSEST_FALLBACK_ENABLED
                or sample_callback is not None
            ):
                raise

            fallback_candidates = self._find_closest_fallback_poses(mc, current, target)
            if not fallback_candidates:
                raise RuntimeError(f"{label} failed and no reachable fallback pose was found.") from exc

            last_fallback_exc: Exception | None = None
            for fallback_pose, ratio in fallback_candidates:
                try:
                    fallback_latest = self._send_pose_target_exact(
                        mc,
                        current,
                        fallback_pose,
                        f"{label} [closest {ratio:.2f}]",
                        rpy_tolerance_deg=rpy_tolerance_deg,
                        speed=speed,
                        sample_callback=None,
                        sample_dt=sample_dt,
                        xyz_tolerance_mm=xyz_tolerance_mm,
                        use_sync=use_sync,
                    )
                except Exception as fallback_exc:
                    if str(fallback_exc) == "Motion aborted by user.":
                        raise
                    last_fallback_exc = fallback_exc
                    continue

                final_xyz_err = self._xyz_distance(fallback_latest[:3], target[:3])
                final_rpy_err = self._max_rpy_error(fallback_latest, target)
                self._set_status(
                    f"{label}: target unreachable, moved to closest pose "
                    f"(xyz err {final_xyz_err:.1f} mm, rpy err {final_rpy_err:.1f} deg)."
                )
                return fallback_latest

            raise RuntimeError(f"{label} failed and all fallback moves failed: {last_fallback_exc}") from exc

    def _send_pose_target_exact(
        self,
        mc: Any,
        current: list[float],
        target: list[float],
        label: str,
        rpy_tolerance_deg: float | None = None,
        *,
        speed: int | None = None,
        sample_callback: Callable[[list[float], list[float], bool], None] | None = None,
        sample_dt: float = 0.08,
        xyz_tolerance_mm: float | None = None,
        use_sync: bool = True,
    ) -> list[float]:
        run_speed = self.SPEED if speed is None else int(speed)
        run_speed = max(1, min(100, run_speed))
        sample_dt_value = float(sample_dt)
        max_rate = sample_dt_value <= 0.0
        poll_dt = 0.0 if max_rate else max(0.01, sample_dt_value)
        xyz_tol = self.XYZ_ARRIVE_TOL_MM if xyz_tolerance_mm is None else max(0.1, float(xyz_tolerance_mm))
        rpy_tol = self.HOME_RPY_ARRIVE_TOL_DEG if rpy_tolerance_deg is None else float(rpy_tolerance_deg)
        dist = self._xyz_distance(current[:3], target[:3])
        timeout_s = min(self.XYZ_MAX_WAIT_S, max(8.0, dist / max(1.0, self.XYZ_MM_PER_SEC_EST) + 8.0))
        self._check_abort(mc)

        if use_sync and sample_callback is None:
            try:
                mc.sync_send_coords(target, run_speed, mode=self.LINEAR_MODE, timeout=timeout_s)
            except Exception:
                self._send_coords_safe(mc, target, self.LINEAR_MODE, run_speed)
        else:
            self._send_coords_safe(mc, target, self.LINEAR_MODE, run_speed)

        deadline = time.time() + timeout_s
        latest_coords = list(current)
        latest_angles = [0.0] * 6
        xyz_err = self._xyz_distance(latest_coords[:3], target[:3])
        rpy_err = max(
            abs(float(latest_coords[3]) - float(target[3])),
            abs(float(latest_coords[4]) - float(target[4])),
            abs(float(latest_coords[5]) - float(target[5])),
        )
        while time.time() < deadline:
            self._check_abort(mc)
            try:
                if sample_callback is None:
                    latest_coords = self._to_six(mc.get_coords(), "get_coords")
                    atomic = False
                else:
                    latest_angles, latest_coords, atomic = self._read_angles_coords_snapshot(mc)
            except Exception:
                if max_rate:
                    time.sleep(self.RECORD_MAX_RATE_FALLBACK_SLEEP_S)
                else:
                    time.sleep(poll_dt)
                continue
            if sample_callback is not None:
                sample_callback(latest_angles, latest_coords, atomic)
            xyz_err = self._xyz_distance(latest_coords[:3], target[:3])
            rpy_err = max(
                abs(float(latest_coords[3]) - float(target[3])),
                abs(float(latest_coords[4]) - float(target[4])),
                abs(float(latest_coords[5]) - float(target[5])),
            )
            if xyz_err <= xyz_tol and rpy_err <= rpy_tol:
                break
            if max_rate:
                time.sleep(0.0)
            else:
                time.sleep(poll_dt)

        if xyz_err > xyz_tol or rpy_err > rpy_tol:
            raise RuntimeError(
                f"{label} incomplete: XYZ error {xyz_err:.1f} mm, "
                f"RPY max error {rpy_err:.1f} deg."
            )
        return latest_coords

    def _find_closest_fallback_poses(
        self,
        mc: Any,
        current: list[float],
        target: list[float],
    ) -> list[tuple[list[float], float]]:
        try:
            current_angles = self._to_six(mc.get_angles(), "get_angles")
        except Exception:
            return []

        total_xyz_delta = self._xyz_distance(current[:3], target[:3])
        total_rpy_delta = self._max_rpy_error(current, target)
        steps = max(3, int(self.CLOSEST_FALLBACK_STEPS))
        max_candidates = max(1, int(self.CLOSEST_FALLBACK_TRY_LIMIT))
        out: list[tuple[list[float], float]] = []

        for i in range(steps, -1, -1):
            ratio = i / steps
            candidate = [
                float(current[k]) + (float(target[k]) - float(current[k])) * ratio
                for k in range(6)
            ]
            candidate, _ = self._clamp_coords(candidate)

            move_xyz = self._xyz_distance(candidate[:3], current[:3])
            move_rpy = self._max_rpy_error(candidate, current)
            if (
                ratio < 1.0
                and (
                    (total_xyz_delta > self.CLOSEST_FALLBACK_MIN_PROGRESS_MM and move_xyz < self.CLOSEST_FALLBACK_MIN_PROGRESS_MM)
                    or (total_rpy_delta > self.CLOSEST_FALLBACK_MIN_PROGRESS_DEG and move_rpy < self.CLOSEST_FALLBACK_MIN_PROGRESS_DEG)
                )
            ):
                continue

            ik_angles = self._solve_ik_angles(mc, candidate, current_angles)
            if ik_angles is None:
                continue
            if out:
                prev_pose = out[-1][0]
                if (
                    self._xyz_distance(prev_pose[:3], candidate[:3]) < 0.5
                    and self._max_rpy_error(prev_pose, candidate) < 0.3
                ):
                    continue
            out.append((candidate, ratio))
            if len(out) >= max_candidates:
                break

        return out

    def _solve_ik_angles(
        self,
        mc: Any,
        target_pose: list[float],
        current_angles: list[float],
    ) -> list[float] | None:
        try:
            solved = mc.solve_inv_kinematics(target_pose, current_angles)
        except Exception:
            return None

        if not isinstance(solved, (list, tuple)) or len(solved) < 6:
            return None

        out: list[float] = []
        try:
            for value in solved[:6]:
                angle = float(value)
                if not isfinite(angle) or abs(angle) > 360.0:
                    return None
                out.append(angle)
        except Exception:
            return None
        return out

    def _move_cartesian_segment(
        self,
        mc: Any,
        start_coords: list[float],
        target_coords: list[float],
        label: str,
        mode: int | None = None,
    ) -> list[float]:
        start_dist = self._xyz_distance(start_coords[:3], target_coords[:3])
        if start_dist <= self.XYZ_ARRIVE_TOL_MM:
            return list(start_coords)

        run_mode = self.LINEAR_MODE if mode is None else int(mode)
        dx = target_coords[0] - start_coords[0]
        dy = target_coords[1] - start_coords[1]
        dz = target_coords[2] - start_coords[2]
        distance = sqrt(dx * dx + dy * dy + dz * dz)
        step_mm = max(1.0, float(self.XYZ_INTERP_STEP_MM))
        hz = max(10.0, float(self.XYZ_STREAM_HZ))
        step_count_dist = int(ceil(distance / step_mm))
        segment_time_s = max(0.5, distance / max(1.0, self.XYZ_MM_PER_SEC_EST))
        step_count_time = int(ceil(segment_time_s * hz))
        num_steps = max(1, step_count_dist, step_count_time, int(self.XYZ_MIN_STREAM_STEPS))
        step_dt = 1.0 / hz
        t0 = time.monotonic()
        report_every = max(1, num_steps // 10)

        def build_waypoint(ratio: float) -> list[float]:
            # Cubic easing removes hard velocity jumps at start/end.
            eased = self._smoothstep(ratio)
            waypoint = [
                round(start_coords[0] + dx * eased, 2),
                round(start_coords[1] + dy * eased, 2),
                round(start_coords[2] + dz * eased, 2),
                target_coords[3],
                target_coords[4],
                target_coords[5],
            ]
            return self._clamp_coords(waypoint)[0]

        for i in range(1, num_steps + 1):
            self._check_abort(mc)
            ratio = i / num_steps
            waypoint = build_waypoint(ratio)
            if i == 1 or i == num_steps or i % report_every == 0:
                self._set_status(f"{label}... step {i}/{num_steps}")
            self._send_coords_safe(mc, waypoint, run_mode)
            if i < num_steps:
                next_tick = t0 + i * step_dt
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)

        try:
            mc.sync_send_coords(target_coords, self.SPEED, mode=run_mode, timeout=6)
        except Exception:
            self._send_coords_safe(mc, target_coords, run_mode)

        timeout_s = min(self.XYZ_MAX_WAIT_S, max(4.0, distance / max(1.0, self.XYZ_MM_PER_SEC_EST)))
        deadline = time.time() + timeout_s
        best_coords = list(start_coords)
        best_dist = start_dist

        while time.time() < deadline:
            self._check_abort(mc)
            try:
                latest = self._to_six(mc.get_coords(), "get_coords")
            except Exception:
                time.sleep(0.08)
                continue
            dist_left = self._xyz_distance(latest[:3], target_coords[:3])
            if dist_left < best_dist:
                best_dist = dist_left
                best_coords = latest
            if dist_left <= self.XYZ_ARRIVE_TOL_MM:
                best_coords = latest
                break
            time.sleep(0.08)

        if best_dist > self.XYZ_ARRIVE_TOL_MM:
            raise RuntimeError(f"{label}: remaining XYZ error {best_dist:.1f} mm.")
        return best_coords

    def _move_axis_incremental(
        self,
        mc: Any,
        current_coords: list[float],
        *,
        axis_idx: int,
        target_value: float,
        label: str,
    ) -> list[float]:
        cur = list(current_coords)
        target = float(target_value)
        start_val = float(cur[axis_idx])
        delta = target - start_val
        if abs(delta) < 0.5:
            return cur

        step_mm = max(1.0, float(self.XYZ_INTERP_STEP_MM))
        steps = max(1, int(ceil(abs(delta) / step_mm)))
        for i in range(1, steps + 1):
            self._check_abort(mc)
            v = round(start_val + delta * (i / steps), 2)
            self._set_status(f"{label}... step {i}/{steps}")
            try:
                mc.send_coord(axis_idx + 1, v, self.SPEED)
            except Exception:
                fallback = list(cur)
                fallback[axis_idx] = v
                fallback, _ = self._clamp_coords(fallback)
                self._send_coords_safe(mc, fallback, self.LINEAR_MODE)
            time.sleep(self.XYZ_INTERP_SETTLE_S)
            try:
                cur = self._to_six(mc.get_coords(), "get_coords")
            except Exception:
                cur[axis_idx] = v
        return cur

    @staticmethod
    def _log_error(message: str, exc: Exception) -> None:
        print(f"[quick_move_gui] ERROR: {message}", flush=True)
        print(f"[quick_move_gui] ERROR TYPE: {type(exc).__name__}", flush=True)
        print("[quick_move_gui] TRACEBACK:", flush=True)
        print(traceback.format_exc(), flush=True)

    def _send_coords_safe(self, mc: Any, coords: list[float], mode: int, speed: int | None = None) -> None:
        run_speed = self.SPEED if speed is None else int(speed)
        run_speed = max(1, min(100, run_speed))
        try:
            mc.send_coords(coords, run_speed, mode)
            return
        except Exception:
            pass
        alt_mode = 0 if int(mode) == 1 else 1
        mc.send_coords(coords, run_speed, alt_mode)

    def _clamp_coords(self, coords: list[float]) -> tuple[list[float], bool]:
        if len(coords) < 6:
            raise RuntimeError(f"coords must have 6 elements, got {coords}")
        out: list[float] = []
        changed = False
        for i in range(6):
            v = float(coords[i])
            lo = float(self.COORD_MIN[i])
            hi = float(self.COORD_MAX[i])
            cv = max(lo, min(hi, v))
            if cv != v:
                changed = True
            out.append(cv)
        return out, changed

    @staticmethod
    def _smoothstep(t: float) -> float:
        t = max(0.0, min(1.0, float(t)))
        return t * t * (3.0 - 2.0 * t)

    @staticmethod
    def _xyz_distance(a_xyz: list[float], b_xyz: list[float]) -> float:
        dx = float(a_xyz[0]) - float(b_xyz[0])
        dy = float(a_xyz[1]) - float(b_xyz[1])
        dz = float(a_xyz[2]) - float(b_xyz[2])
        return sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _max_rpy_error(a_pose: list[float], b_pose: list[float]) -> float:
        return max(
            abs(float(a_pose[3]) - float(b_pose[3])),
            abs(float(a_pose[4]) - float(b_pose[4])),
            abs(float(a_pose[5]) - float(b_pose[5])),
        )

    def _maximize_window(self) -> None:
        try:
            self.state("zoomed")
            return
        except Exception:
            pass
        try:
            self.attributes("-zoomed", True)
            return
        except Exception:
            pass
        width = int(self.winfo_screenwidth())
        height = int(self.winfo_screenheight())
        self.geometry(f"{width}x{height}+0+0")

    def _toggle_fullscreen(self, _: tk.Event | None = None) -> None:
        try:
            enabled = bool(self.attributes("-fullscreen"))
            self.attributes("-fullscreen", not enabled)
        except Exception:
            pass

    def _exit_fullscreen(self, _: tk.Event | None = None) -> None:
        try:
            self.attributes("-fullscreen", False)
        except Exception:
            pass

    def _on_close(self) -> None:
        if self.mc is not None:
            try:
                self.mc.close()
            except Exception:
                pass
        self.destroy()

    @staticmethod
    def _format_value(value: float) -> str:
        text = f"{value:.2f}"
        if "." in text:
            text = text.rstrip("0").rstrip(".")
        return text


def main() -> None:
    app = QuickMoveGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
