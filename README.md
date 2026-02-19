# MyCobot Control Toolkit

Python tools for controlling a MyCobot 280 robot arm:

- `manipulation_degrees/quick_move_gui.py`: main GUI for quick jog/move, saved poses, sequence building, vacuum control, and CSV motion recording.
- `motion_gui.py` + `record_motion.py`: older motion recording/playback utilities.
- `Cube_detection_pickup/`: camera-based cube detection and pick/place scripts.

## Requirements

- Python 3.10+ recommended
- MyCobot 280 connected over USB serial
- Linux users: `tkinter` package installed (`python3-tk`)

Python packages are listed in `requirements.txt`:

- `pymycobot`
- `numpy`
- `opencv-python`

## Installation

### 1. Clone repository

```bash
git clone <your-repo-url>.git
cd MyCobot
```

### 2. (Recommended) create virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install system packages (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y python3-tk
```

### 4. Install Python dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

## Quick Start (Main GUI)

Run:

```bash
python manipulation_degrees/quick_move_gui.py
```

Default serial settings:

- Port: `/dev/ttyACM0`
- Baudrate: `115200`

You can also override defaults with environment variables:

```bash
export MYCOBOT_PORT=/dev/ttyACM0
export MYCOBOT_BAUD=115200
```

Settings are saved to:

- `manipulation_degrees/quick_move_settings.json`

## Main Features (quick_move_gui)

- Joint and Cartesian jog control (`+/-` and direct numeric input)
- Move to target pose (`x, y, z, rx, ry, rz`)
- Named pose save/load/edit
- Sequence builder:
  - Move named pose
  - Home
  - Wait
  - Vacuum ON/OFF
  - Reorder, copy/paste, delete
- Sequence save/load (`.json`)
- CSV recording while sequence runs (step-based snapshot mode)
- Abort motion
- Home/target closest-reachable fallback when exact target is not reachable
- Adjustable in-position tolerances in Settings

## Other Scripts

### Legacy motion GUI

```bash
python motion_gui.py
```

### Camera preview / detection utilities

```bash
python Cube_detection_pickup/camera_preview.py
python Cube_detection_pickup/main.py
```

## Notes

- Robot control and camera control are hardware-dependent; always test with low speed first.
- Verify vacuum wiring matches the configured digital pins in code.
- If serial permissions fail on Linux, add your user to `dialout` and re-login:

```bash
sudo usermod -aG dialout $USER
```

