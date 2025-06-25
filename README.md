# 🎯 Interactive PID Controller Demo (PyBullet + Python)

This project demonstrates Proportional-Integral-Derivative (PID) control using Python and PyBullet. It's a teaching tool that shows how a cube moves to target positions based on live-tuned PID gains.

## 🚀 Features

- Real-time PID tuning via GUI sliders.
- Visual animation of a cube moving to a target.
- Side-by-side plots showing overshoot vs tuned response.
- 2D scattered target movement using tuned PID.

## 📂 Files

- `pid_single_run_gui.py`: Single-run PID with live sliders.
- `pid_two_stage_2d.py`: Multi-target 2D demo comparing overshoot vs tuned control.

## 📦 Requirements

```bash
pip install pybullet matplotlib
```

## 🧠 Usage

```bash
python pid_single_run_gui.py
python pid_two_stage_2d.py
```

## 🧪 Learning Goals

- Understand how PID components affect motion.
- Observe overshoot, damping, and steady-state error.
- Tune and compare gains for real-time effect.
