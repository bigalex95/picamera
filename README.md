# picamera

Scripts from a small Raspberry Pi toy-car project (2021): lane following, red traffic-light stop and obstacle distance reading. Code comments and file names are in Turkish. 2021 experiment, not maintained.

## What's inside

All files are flat in the repo root.

| Files | Purpose |
|---|---|
| `serittakibi_1.py` | Lane following (Canny edges, `cv2.HoughLinesP`, steering from line angle) |
| `trafiklambasi_1.py`, `redStop.py`, `redStopv1-v3.py` | Red traffic-light detection with an HSV colour mask |
| `serihaberlesme_1.py` | Reads distance values from an Arduino/ultrasonic sensor over serial |
| `shapedetector.py` | Contour-based shape detection example |
| `mainv1.py` ... `mainv5.py` | Successive iterations combining lane following, traffic light, serial obstacle check and GPIO motor control; `mainv5.py` is the latest |
| `anafonksiyon.txt`, `trafiklambasi_2.txt`, `trafikson.txt`, `ustaa.txt` | Code snapshots saved as text files |

## Run

Needs a Raspberry Pi with the camera module and the old `picamera` stack, plus motors wired to GPIO:

```bash
pip install opencv-python numpy imutils pyserial picamera RPi.GPIO
python3 mainv5.py
```

I did not run this (no hardware); there is no requirements file and the pin numbers and serial port are hard-coded in the scripts.

## Status

Prototype from 2021, kept for reference. The `mainv*` files are copies of each other at different stages, not a package.

## Credits

Written by me with standard OpenCV techniques; `shapedetector.py` uses a common tutorial pattern (contour approximation).
