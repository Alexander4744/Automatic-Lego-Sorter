# Automatic LEGO Sorter

A fully integrated system for detecting and automatically sorting LEGO bricks based on their shape using YOLO11 computer vision and embedded real-time control.

[Complete LEGO Sorting System](../Images/Complete%20Sorting%20Line.png)

## Project Overview

This project implements an end-to-end automated LEGO sorting system combining:
- **Mechanical transport**: Dual conveyor belt system with servo-controlled sorter
- **Real-time control**: STM32 microcontroller managing sensors and actuators
- **Computer vision**: YOLO11 neural network for brick classification
- **Edge inference**: Raspberry Pi 5 running inference and decision logic

The system can detect and classify up to 5 types of LEGO bricks (currently deployed; architecture supports up to 9) and sort them into corresponding bins.

## System Architecture

### Three Main Subsystems

1. **Mechanical System** (Transport & Sorting)
   - Dual DC motor-driven conveyor belts
   - Pneumatic separators on each belt
   - Servo-controlled distribution manifold
   - 5 output bins for sorted bricks

2. **Real-Time Control** (STM32-based)
   - Motor speed regulation via PWM
   - Ultrasonic sensor processing for brick detection/confirmation
   - Text-based protocol communication with Raspberry Pi
   - 10-second inter-capture cooldown (avoids duplicate triggers)
   - 10-second pâlnie repositioning delay (mechanical settling)

3. **AI/Computer Vision** (Raspberry Pi 5)
   - YOLO11 model for real-time brick classification
   - Python inference pipeline with fallback robustness
   - 5-second timeout for classification requests (filters false triggers)
   - Asynchronous communication with STM32 via serial

## Key Components

### Hardware
- **STM32F401RE**: Main microcontroller (motor control, sensor I/O)
- **Raspberry Pi 5**: Computer vision inference
- **Motors**: Two 12V DC motors (conveyor belts)
- **Sensors**: Two HC-SR04 ultrasonic sensors (camera trigger, pâlnie confirmation)
- **Actuators**: Standard servo for distribution manifold
- **Camera**: Raspberry Pi Camera Module (for brick imaging)

### Software
- **Firmware**: STM32 C code with event-driven loop architecture
- **Application**: Python 3 on Raspberry Pi (PyQt-based GUI optional)
- **Model**: YOLO11 trained on project-specific LEGO dataset

## Communication Protocol

**STM32 ↔ Raspberry Pi**: Text-based protocol over serial (115200 baud)

### Commands (Pi → STM32)
- `CMRA\n` — Capture image (trigger camera)
- `SORT <BIN>\n` — Route next brick to specified bin (0–4)

### Events (STM32 → Pi)
- `EVT READY\n` — System initialized and ready
- `EVT SORTED\n` — Brick successfully dispatched
- `EVT QUEUE_FULL\n` — Input queue capacity reached
- `<CLASS_NAME>\n` — Classification result from Pi → STM32 (e.g., `1x2 Bricks\n`)
- `ERR\n` — Classification failed or timeout

## System Characteristics

### Performance
- **Throughput**: ~1 brick per 15–20 seconds (limited by mechanical speed and settling times)
- **Accuracy**: Good across 5 classes (see Known Issues)
- **Detection range**: 60–180 mm (camera trigger), 120–160 mm (pâlnie confirmation)

### Design Rationale
The 10-second cooldowns and timeouts are **not limitations but requirements**:
- **Camera cooldown** prevents spurious re-triggers on the same brick
- **Pâlnie delay** ensures mechanical settling before next dispatch
- **Pi timeout** filters electrical noise and false object detection

These timings reflect the physical constraints of the motor/belt system and would scale with faster hardware.

## Known Issues & Mitigations

### Issue: Brick Misclassification
- **Symptom**: 1×4 bricks confused with 2×4; 1×2 confused with 2×2
- **Root cause**: Training dataset captured from single viewpoint; bricks appear identical in that orientation
- **Mitigation**: Use the system to auto-generate training data from multiple angles, manually correct labels, and retrain

### Issue: Mechanical Speed Bottleneck
- **Symptom**: Throughput limited to ~1 brick per 15–20 seconds
- **Root cause**: Motor/belt design; settling time inherent to servo-based distribution
- **Mitigation**: Upgrade to higher-speed motors and smooth-running belt system; this would eliminate cooldown requirements

## Project Structure

```
Automatic-Lego-Sorter/
├── README.md
├── stm32/                    # STM32F401RE firmware
│   ├── main.c
│   ├── motor_control.c/h
│   ├── sensor.c/h
│   └── communication.c/h
├── raspberry_pi/             # Raspberry Pi application
│   ├── main.py
│   ├── inference.py
│   ├── communication.py
│   ├── gui.py (optional)
│   └── yolo11_model/
└── docs/
    ├── MECHANICAL.md         # Detailed mechanical subsystem
    ├── REALTIME_CONTROL.md   # STM32 firmware & protocol
    └── AI_COMPUTER_VISION.md # YOLO11 training & inference
```

## Getting Started

### Prerequisites
- STM32CubeIDE or compatible toolchain
- Python 3.10+ (Raspberry Pi)
- OpenCV, PyTorch, Ultralytics (Pi requirements)
- LEGO bricks for testing

### Deployment Steps
1. **Flash STM32 firmware** onto microcontroller
2. **Configure serial connection** (STM32 UART ↔ Raspberry Pi GPIO)
3. **Place trained YOLO11 model** in `raspberry_pi/yolo11_model/`
4. **Run Raspberry Pi application**: `python3 main.py`
5. **Power up motor system**; STM32 sends `EVT READY`
6. **Feed bricks** into input conveyor; system auto-triggers and sorts

### Configuration
- **Serial baudrate**: 115200 bps (adjustable in both STM32 and Pi code)
- **Servo angles**: Configured per-bin in STM32 (currently 0°, 37°, 74°, 111°, 148°, 180°; note: servo is uncalibrated, physical bins align with 0°, 35°, 70°, 105°, 140°, 175°)
- **Sensor thresholds**: Camera 60–180 mm; Pâlnie 120–160 mm (in `sensor.c`)
- **Motor PWM**: Configurable in firmware (default 80% speed)

## Testing & Validation

### Unit Testing
- STM32 loop tested with serial terminal (manual commands)
- Python inference validated offline on sample images

### Integration Testing
- End-to-end sorting of 5 LEGO types
- Serial protocol stress-tested with rapid commands
- Timeout behavior verified under Pi inference delays

### Known Test Limitations
- Limited test dataset (~50 bricks per type)
- Single-angle training data (explains misclassification for viewpoint-ambiguous pairs)

## Future Improvements

1. **Increase throughput**: Upgrade motor/belt to faster, smoother system
2. **Improve accuracy**: Retrain YOLO11 with multi-angle dataset
3. **Formal state machine**: Replace boolean flag loop with explicit FSM (optional, current implementation stable)
4. **Expand capacity**: Support 6+ brick types (firmware architecture ready)
5. **Fault tolerance**: Add watchdog timer and error recovery sequences

## Documentation

For detailed technical information, see:
- **[MECHANICAL.md](./docs/MECHANICAL.md)** — Conveyor design, servo mechanics, sensors
- **[REALTIME_CONTROL.md](./docs/REALTIME_CONTROL.md)** — STM32 architecture, protocol, timing
- **[AI_COMPUTER_VISION.md](./docs/AI_COMPUTER_VISION.md)** — YOLO11 training, dataset, inference

## License

[Specify your license here, e.g., MIT, GPL, etc.]

## Author

Alexander4744

## Acknowledgments

- YOLO11 by Ultralytics
- Raspberry Pi Foundation
- STM32 community
