# UNO Q Fruit Sorting Robot

A safety-oriented, production-structured fruit sorting application for the **Arduino UNO Q 4 GB**:

- Qualcomm/Linux side: Logitech C270 camera, YOLOv8 inference, ByteTrack object tracking, colour validation, target selection, calibration, kinematics, planning, logging, and state management.
- STM32/real-time side: PCA9685 control, smooth six-servo interpolation, joint-limit enforcement, physical emergency-stop input, output-enable control, and heartbeat watchdog.
- Supported fruits only:
  - Apple, verified red
  - Banana, verified yellow
  - Orange, verified orange

The application ignores all other YOLO classes.

## Important limitations

This project is a strong engineering starting point, but it cannot be mechanically universal because 6-DOF arms differ in link lengths, servo orientation, horn installation, joint limits, and reachable workspace.

Before enabling automatic picking, you must calibrate:

1. Servo directions, pulse limits, and kinematic zero offsets.
2. Arm link lengths and base height.
3. Camera-to-table homography.
4. Fruit pick heights and box coordinates.
5. Safe travel height and keep-out zones.

`calibration_complete` is `false` by default. The program will home the robot and run vision, but will not execute an automatic pick until calibration is explicitly completed.

A single fixed webcam does not provide true depth. This design assumes all fruit lies on a known flat work surface. Camera pixels are converted to robot X/Y using a planar homography, while fruit Z is configured per fruit type.

Software collision avoidance cannot detect unexpected people, cables, loose objects, or a mechanically shifted arm. A real installation needs guarding, a power-rated emergency stop, risk assessment, and validation by a qualified engineer.

## Project layout

```text
uno_q_fruit_sorter/
├── config/robot.yaml
├── python/main.py
├── python/fruit_sorter/
│   ├── calibration.py
│   ├── calibrate_workspace.py
│   ├── camera.py
│   ├── colour.py
│   ├── config.py
│   ├── controller.py
│   ├── detector.py
│   ├── dashboard.py
│   ├── exceptions.py
│   ├── kinematics.py
│   ├── logging_setup.py
│   ├── main.py
│   ├── models.py
│   ├── robot_client.py
│   ├── robot_rpc.py
│   ├── safety.py
│   ├── tracking.py
│   ├── validate_config.py
│   └── web/
│       ├── index.html
│       ├── styles.css
│       └── app.js
├── sketch/sketch.ino
├── systemd/fruit-sorter.service
├── tests/
├── pyproject.toml
└── requirements.txt
```

## Wiring

### PCA9685 logic and servos

| UNO Q / Supply | PCA9685 |
|---|---|
| UNO Q 3.3 V | VCC / logic supply |
| UNO Q GND | GND |
| UNO Q D20 / SDA | SDA |
| UNO Q D21 / SCL | SCL |
| External regulated 6 V positive | V+ |
| External regulated 6 V negative | GND |
| UNO Q D3 | OE, active-low output enable |

Connect all grounds together.

Connect the six servos to PCA9685 channels 0–5 by default:

| Channel | Joint |
|---:|---|
| 0 | Base |
| 1 | Shoulder |
| 2 | Elbow |
| 3 | Wrist pitch |
| 4 | Wrist roll |
| 5 | Gripper |

Do not power the servos from the UNO Q. The external 6 V supply must be sized for the combined servo stall current, with suitable wiring, fuse, and bulk capacitance.

### Emergency stop

For fail-safe input monitoring, use a normally-closed auxiliary contact:

- D2 configured as `INPUT_PULLUP`.
- D2 connected to GND through the normally-closed auxiliary contact.
- Normal condition: D2 is LOW.
- Pressed, disconnected, or broken wire: D2 becomes HIGH and the MCU disables PCA9685 OE.

For a real emergency stop, the mushroom switch's power contacts must also interrupt the external 6 V servo supply through appropriately rated safety hardware. The D2 input and software watchdog are secondary controls; they are not a substitute for removing actuator power.

### Logitech C270

Connect the C270 to the UNO Q USB-C port through a suitable USB host hub/dongle. A powered hub is recommended when other USB devices are attached.

## UNO Q software architecture

The project uses the UNO Q Router Bridge Unix socket:

```text
/var/run/arduino-router.sock
```

The Linux Python process calls functions exposed by the STM32 sketch. This keeps AI work on Linux and deterministic actuator/safety functions on the MCU.

## Installation

### 1. Flash the STM32 sketch

Open this project in Arduino App Lab and flash:

```text
sketch/sketch.ino
```

The sketch requires the UNO Q core and `Arduino_RouterBridge`, both supplied through the UNO Q development environment.

### 2. Install Linux packages

From the UNO Q Debian terminal:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip python3-opencv python3-msgpack
```

Create the virtual environment:

```bash
cd /path/to/uno_q_fruit_sorter
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`opencv-python` is intentionally not forced in `requirements.txt`, because Debian's `python3-opencv` is often simpler on ARM64. Install it with pip only if your environment requires it.

The first run of `yolov8n.pt` may download model weights. For an offline deployment, download the model once and set `detector.model` to its local absolute path.

### 3. Validate the configuration

```bash
PYTHONPATH=python .venv/bin/python -m fruit_sorter.validate_config --config config/robot.yaml
```

### 4. Test in dry-run mode

```bash
PYTHONPATH=python .venv/bin/python -m fruit_sorter.main \
  --config config/robot.yaml \
  --dry-run
```

Dry-run performs camera, YOLO, colour, tracking, mapping, kinematics, and planning while simulating servo movement.

## Calibration

### A. Servo calibration

With the servo power disconnected:

1. Mechanically place each joint near its intended neutral position.
2. Install the horns.
3. Restore power with the arm supported.
4. Confirm that `[90, 90, 90, 90, 90, 90]` is a safe home pose.
5. Adjust each servo's:
   - `min_deg`
   - `max_deg`
   - `min_pulse_us`
   - `max_pulse_us`
   - `kinematic_zero_deg`
   - `direction`

Never use broad 0–180° limits unless the joint has physically been proven safe across that range.

### B. Camera homography

Mount the camera rigidly above the table.

The four `robot_reference_points_mm` values in the YAML are known positions in the robot frame. Place visible marks at those positions and run:

```bash
PYTHONPATH=python .venv/bin/python -m fruit_sorter.calibrate_workspace \
  --config config/robot.yaml
```

Click the four marks in the displayed order. The tool writes the homography into the YAML.

### C. Arm geometry and pick/drop coordinates

Measure and update:

- `base_height_mm`
- `upper_arm_mm`
- `forearm_mm`
- `tool_mm`
- `tool_pitch_deg`
- workspace limits
- keep-out zones
- per-fruit `pick_z_mm`
- box X/Y/release Z

Run the validation tool after every change.

### D. Enable automatic movement

Only after dry-run and empty-gripper testing:

```yaml
calibration:
  calibration_complete: true
```

## Running

```bash
PYTHONPATH=python .venv/bin/python -m fruit_sorter.main \
  --config config/robot.yaml
```

Keyboard controls when the preview window is enabled:

- `E`: emergency stop
- `Q`: controlled shutdown

The program also handles SIGINT and SIGTERM. The web dashboard remains active when `--no-display` disables the local OpenCV window.


## Web dashboard

The dashboard starts automatically with the robot and is enabled by default:

```yaml
web:
  enabled: true
  host: "0.0.0.0"
  port: 8000
  stream_fps: 10.0
  jpeg_quality: 75
```

On the UNO Q, find the board's IP address:

```bash
hostname -I
```

Open the following address on the monitor or another device connected to the same network:

```text
http://UNO_Q_IP_ADDRESS:8000
```

Examples:

```text
http://192.168.1.45:8000
http://localhost:8000
```

Use `localhost` when the browser is running directly on the UNO Q desktop.

The dashboard shows:

- Live annotated Logitech C270 camera stream.
- Apple, banana and orange bounding boxes.
- YOLO confidence and ByteTrack ID.
- Colour-match percentage and validation result.
- Stable/moving state and the final pick decision.
- Pixel centre and calibrated robot X/Y position.
- Robot state: idle, acquiring, picking, placing, homing or stopped.
- Processing FPS and inference time.
- Current six-servo angles.
- PCA9685 output status and motion status.
- MCU fault and physical emergency-stop status.
- Per-fruit and total sorted counts.
- Current target, last completed sort and cycle time.
- Recent robot events and errors.

The JSON telemetry is also available at:

```text
http://UNO_Q_IP_ADDRESS:8000/api/status
```

The annotated MJPEG stream is available at:

```text
http://UNO_Q_IP_ADDRESS:8000/video
```

### Optional browser emergency-stop button

Remote control is disabled by default. The physical power-cut emergency stop remains the primary safety device.

To enable the dashboard's software E-stop button, use a random token containing at least 12 characters:

```yaml
web:
  allow_remote_estop: true
  control_token: "replace-with-a-long-random-token"
```

After restarting the service, enter the same token on the dashboard before pressing **Emergency stop**. Do not expose port 8000 to the public internet.

To disable the browser dashboard for one run:

```bash
PYTHONPATH=python .venv/bin/python -m fruit_sorter.main \
  --config config/robot.yaml \
  --no-web
```

## Sorting sequence

For a stable, colour-verified, unpicked tracked fruit:

1. Convert camera centre pixel to robot X/Y.
2. Check workspace bounds and keep-out zones.
3. Solve inverse kinematics and validate all joint limits.
4. Open gripper.
5. Move above fruit.
6. Lower in sampled Cartesian steps.
7. Close gripper.
8. Lift to safe travel height.
9. Move laterally at safe height.
10. Lower into the configured fruit box.
11. Open gripper.
12. Lift.
13. Return all six servos to 90°.
14. Record track ID and pick location to avoid duplicate picking.

## Same-fruit prevention

Two layers are used:

- The current ByteTrack ID is remembered after a completed sort.
- The original robot X/Y pick location is remembered for a configurable time and radius.

The spatial memory protects against tracker ID changes after temporary occlusion.


## Idle watchdog behaviour

The Linux process runs a dedicated heartbeat thread for the entire application lifetime, including while the arm is idle and waiting for fruit. If Linux, the Python process or Router Bridge stops responding, the MCU watchdog disables PCA9685 output through the `OE` pin.

## Logging

Logs are written to:

```text
logs/fruit_sorter.log
```

A rotating handler limits log file growth.

## Service installation

Edit `systemd/fruit-sorter.service` for your username and installation path, then:

```bash
sudo cp systemd/fruit-sorter.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now fruit-sorter.service
```

Check logs:

```bash
journalctl -u fruit-sorter.service -f
```

## Model and licence note

This project code is provided under the MIT licence. Ultralytics and the YOLO model have their own licence terms. Confirm that your deployment model and commercial use comply with the applicable Ultralytics licence.

## Official references

- Arduino UNO Q: https://docs.arduino.cc/hardware/uno-q
- UNO Q user manual and Router Bridge: https://docs.arduino.cc/tutorials/uno-q/user-manual/
- Ultralytics tracking: https://docs.ultralytics.com/modes/track/
- Ultralytics COCO classes: https://docs.ultralytics.com/datasets/detect/coco/
