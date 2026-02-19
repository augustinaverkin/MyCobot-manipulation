# Quick Move GUI (MyCobot 280)

This project provides a single desktop application for controlling a MyCobot 280:

- `quick_move_gui.py`

The app includes manual jog control, target pose moves, saved named poses, sequence building, vacuum control, and CSV motion recording.

## Requirements

- Python 3.10+
- MyCobot 280 connected via USB serial
- Linux: `python3-tk` installed

Python packages:

- `pymycobot`
- `numpy`
- `opencv-python`

## Installation

### 1. Create and activate a virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 2. Install system dependency (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y python3-tk
```

### 3. Install Python dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

## Run

From this folder:

```bash
python quick_move_gui.py
```

Default serial connection:

- Port: `/dev/ttyACM0`
- Baudrate: `115200`

Optional environment overrides:

```bash
export MYCOBOT_PORT=/dev/ttyACM0
export MYCOBOT_BAUD=115200
```

## Main Features

- Joint and Cartesian quick move controls (`+/-` and direct numeric input)
- Move to target pose (`x, y, z, rx, ry, rz`)
- Named position management
- Sequence builder with:
  - Move to named pose / home
  - Wait step
  - Vacuum ON/OFF step
  - Reorder, delete, clear
  - Copy/paste step
- Per-step move pose editing in sequence
- Sequence save/load (`.json`)
- CSV recording by reached step
- Abort motion button
- Closest-reachable fallback for unreachable targets
- Adjustable in-position tolerances in Settings

## Settings File

The GUI stores settings in:

- `quick_move_settings.json`

This includes serial port/baudrate, home pose, tool length, and tolerance values.

## Notes

- Start with low speeds for first tests.
- If serial access is denied on Linux:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in.

