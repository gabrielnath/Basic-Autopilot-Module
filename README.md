# Basic-Autopilot-Module
A small, serious-ish autopilot module written in **Ada**.

>No, it's not flying a real jet.


## Overview
This project is a **real-time autopilot control module** written in **Ada**, designed around classic **PID control loops** for aircraft control.

Unlike my **last two projects**, which were pure simulations, this one is written as a **real module**, structured the way an actual embedded control task would be:

- Uses **Ada.Real_TIme**
- Organized as a real-time control loop
- Clear separation between **sensors, actuators, control logic**, and **safety handling**

>Is it flying a real jet?  
**No**.

>Is it structured like something that could *eventually* belong in a real system?  
**That's the goal  (at least I hope so)**

## Project Features
- Autopilot control for altitude, heading, speed
- **Discrete PID controllers** for each control axis
- Sensor redundancy (primary/ secondary)
- Exception handling for safety-critical paths
- Actuator output limiting safe ranges
- Real-time control loop with deterministic timing

### Control Theory Overview

This project uses classic PID (Proportional–Integral–Derivative) control, implemented in discrete time.

- The **proportional term** reacts to the current error.
- The **integral term** accumulates past error to remove steady-state offset.
- The **derivative term** reacts to how fast the error is changing.

Because this is a digital system, the controller operates in discrete time:
- Integrals are approximated by summing error over control cycles.
- Derivatives are approximated using the difference between successive errors.

This is the standard approach used in embedded and aerospace control software.


## Why Ada?
Because:

- Ada is widely used in **safety-critical aerospace systems**
- Strong typing + explicitness = fewer “oops” moments
- I wanted to suffer productively and learn something valuable

I’m still **new to Ada** and **aerospace software**, so if you see something that looks like:

> "This works, but there’s probably a better Ada-ish way"

You're probably right.

## How to Run

You’ll need GNAT to compile and run the Ada version of the project. Here’s how to get started:

1. Install GNAT if you don’t have it.
2. Compile and run the Ada version:

```bash
gnatmake Basic_Autopilot.adb
./Basic_Autopilot    # Windows: Basic_Autopilot.exe
```

## Notes & Limitations
- Certified avionics systems typically **avoid exception-driven control flow**.
- Status codes and health monitors are preferred.
- Control gains are **not tuned for any real aircraft**
- No aerodynamic model is included
- No redundancy voting logic is implemented
- This is **not flight-certified software**