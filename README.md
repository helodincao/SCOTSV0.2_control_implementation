# SCOTSV0.2_control_implementation

## Overview

This project rebuilds the Go2 robot controller using SCOTS v0.2.

The main goal is to:

1. Generate a controller with SCOTS.
2. Test that controller in simulation.
3. Make the controller available to Python.
4. Build a deployment loop that can later connect to the real Go2 robot and OptiTrack.

The project is designed so that the same controller logic can be tested on a Mac using Docker before being used on the real lab hardware.

---

# Project Structure

The current structure is:

```text
SCOTSV0.2_control_implementation/
│
├── go2_v02/
│   ├── synthesis/
│   │   ├── go2_controller.cc
│   │   ├── go2_model.hh
│   │   ├── arena_config.txt
│   │   ├── Makefile
│   │   ├── controller.scs
│   │   └── target.scs
│   │
│   ├── sim/
│   │   ├── simulate.cc
│   │   └── Makefile
│   │
│   ├── deploy/
│   │   ├── scots_binding.cc
│   │   ├── test_binding.py
│   │   ├── sim_python.py
│   │   ├── pose_source.py
│   │   ├── command_sink.py
│   │   └── deploy_loop.py
│   │
│   └── docker/
│       ├── Dockerfile.synth
│       └── Dockerfile.deploy
│
├── src/
└── utils/
```

---

# 1. Docker Setup

Docker is used so the project can run in the same environment on different computers.

This avoids depending on whatever C++, Python, or library versions happen to already be installed on a machine.

## Synthesis Container

The synthesis container uses Ubuntu and installs the C++ tools needed to compile SCOTS.

`go2_v02/docker/Dockerfile.synth`:

```dockerfile
FROM ubuntu:24.04

RUN apt-get update && \
    apt-get install -y g++ make

WORKDIR /workspace

CMD ["bash"]
```

Build it with:

```bash
docker build -f go2_v02/docker/Dockerfile.synth -t scots-synth .
```

Run it with:

```bash
docker run --rm -it \
  --mount type=bind,source="$(pwd)",target=/workspace \
  scots-synth
```

The bind mount makes the project folder on the computer available inside Docker at:

```text
/workspace
```

Changes made inside that folder are also saved on the computer.

---

# 2. Starting from the SCOTS Vehicle Example

The SCOTS v0.2 vehicle example was used as the starting point for the new Go2 controller.

The original vehicle example was copied into:

```text
go2_v02/synthesis/go2_controller.cc
```

This gave us a known-working SCOTS v0.2 program before changing anything for the Go2.

The original example successfully produced a winning controller, which confirmed that SCOTS v0.2 was compiling and running correctly.

---

# 3. Go2 Motion Model

The example vehicle dynamics were replaced with the Go2 motion model.

The robot state is:

```text
[x, y, yaw]
```

where:

* `x` is the robot's x position
* `y` is the robot's y position
* `yaw` is the robot's heading

The controller input is:

```text
[vx, vy, vyaw]
```

where:

* `vx` is forward/backward body velocity
* `vy` is sideways body velocity
* `vyaw` is turning speed

The model is:

```text
dx/dt = vx*cos(yaw) - vy*sin(yaw)

dy/dt = vx*sin(yaw) + vy*cos(yaw)

dyaw/dt = vyaw
```

These commands are body-frame commands.

That matters later because the Go2 command:

```python
SportClient.Move(vx, vy, vyaw)
```

also expects body-frame velocities.

The controller output should therefore be sent directly to the robot without rotating it again.

---

# 4. RK4 Integration

SCOTS needs to predict how the robot moves after a command is applied.

The project uses SCOTS's RK4 integration code:

```cpp
scots::runge_kutta_fixed4(...)
```

The current settings are:

```text
tau = 0.3 seconds
nint = 10
```

This means each controller action represents 0.3 seconds of movement, divided into 10 smaller integration steps.

Each smaller step is:

```text
0.3 / 10 = 0.03 seconds
```

---

# 5. State Grid

The current real test arena uses:

```text
x:   -1.5 to 1.5
y:   -1.5 to 1.5
yaw: -3.5 to 3.5
```

with grid spacing:

```text
x:   0.1
y:   0.1
yaw: 0.2
```

This creates:

```text
31 × 31 × 35 = 33,635 states
```

SCOTS checks these possible states when building the controller.

---

# 6. Input Grid

The current robot command limits are:

```text
vx:   -0.5 to 0.76
vy:   -0.2 to 0.2
vyaw: -0.8 to 0.8
```

The spacing between possible commands is:

```text
vx:   0.25
vy:   0.2
vyaw: 0.8
```

This gives:

```text
6 × 3 × 3 = 54 possible commands
```

An earlier version used a much larger `vx` spacing of `0.63`.

That only produced two possible forward velocity values and removed useful motion options.

With that coarse input grid, SCOTS found no winning solution.

Changing the `vx` spacing to:

```text
0.25
```

gave SCOTS enough possible commands to find a working controller.

---

# 7. Shared Go2 Model

Common Go2 code was moved into:

```text
go2_v02/synthesis/go2_model.hh
```

This prevents the synthesis code and simulation code from accidentally using different robot models.

The shared file contains:

* state and input types
* arena configuration
* target information
* configuration file parsing
* Go2 dynamics
* RK4 movement
* target checking

Both synthesis and simulation now use the same model.

---

# 8. Arena Configuration File

Instead of hardcoding the arena inside the C++ program, most settings are stored in:

```text
go2_v02/synthesis/arena_config.txt
```

The current configuration is:

```text
tau 0.3
nint 10

state_lb -1.5 -1.5 -3.5
state_ub  1.5  1.5  3.5
state_eta 0.1 0.1 0.2

input_lb -0.5 -0.2 -0.8
input_ub  0.76 0.2 0.8
input_eta 0.25 0.2 0.8

target 1.2 0.0 0.0 3.3 3.3 0.001

obstacle -0.55 -0.45 -1.5 0.65
obstacle 0.15 0.25 -0.65 1.5
obstacle 0.75 0.85 -1.5 0.55
```

This means the arena can be changed without rewriting the controller source code.

---

# 9. Obstacles

The current arena contains three rectangular obstacles.

SCOTS marks states inside these rectangles as unsafe.

The obstacle checks also account for half of a grid cell around the obstacle edges so the grid representation is handled correctly.

The three obstacles create a slalom-like path through the arena.

---

# 10. Target

The target is represented as an ellipse around:

```text
x = 1.2
y = 0.0
```

The current target configuration is:

```text
target 1.2 0.0 0.0 3.3 3.3 0.001
```

The yaw value is given a very small weight, so the robot mainly needs to reach the correct position.

It does not need to finish at one exact heading.

---

# 11. Controller Synthesis

The C++ synthesis program:

```text
go2_v02/synthesis/go2_controller.cc
```

uses the arena configuration and Go2 model to calculate a safe controller.

With the current arena:

```text
States: 33,635
Inputs: 54
Transitions: about 12 million
Winning states: 25,515
```

A winning state is a state from which SCOTS has found a safe way to eventually reach the target.

The final controller is saved as:

```text
controller.scs
```

The target is also saved as:

```text
target.scs
```

---

# 12. C++ Simulation

A separate C++ simulator was created:

```text
go2_v02/sim/simulate.cc
```

The simulator:

1. Starts the robot at:

```text
[-1.3, 0.0, 0.0]
```

2. Loads `controller.scs`.

3. Asks the controller for a command.

4. Applies that command using the same Go2 model used during synthesis.

5. Repeats until the target is reached.

The simulated robot successfully followed the slalom and reached the target in 40 controller steps.

This was the first confirmation that the generated controller worked.

---

# 13. Python Binding for SCOTS

The existing Go2 deployment code is mostly Python.

SCOTS v0.2 is C++.

To allow Python to use the SCOTS controller directly, a small pybind11 wrapper was created:

```text
go2_v02/deploy/scots_binding.cc
```

The Python-facing class is:

```python
Controller
```

Python can now do:

```python
controller = scots_ctrl.Controller("controller")

controls = controller.get_control([x, y, yaw])
```

and receive possible Go2 commands.

The binding uses:

```cpp
peek_control(...)
```

instead of the normal SCOTS `get_control(...)`.

This is useful for deployment because if the robot leaves the winning set, it returns no command instead of forcing a controller lookup that may fail.

---

# 14. Deployment Docker Container

A second Docker image was created for Python deployment testing.

`go2_v02/docker/Dockerfile.deploy` installs:

* C++
* Python
* Python development files
* pybind11

It can be built with:

```bash
docker build -f go2_v02/docker/Dockerfile.deploy -t scots-deploy .
```

and run with:

```bash
docker run --rm -it \
  --mount type=bind,source="$(pwd)",target=/workspace \
  scots-deploy
```

---

# 15. Testing the Python Binding

A small test was created:

```text
go2_v02/deploy/test_binding.py
```

Using the starting state:

```text
[-1.3, 0.0, 0.0]
```

the Python binding returned:

```text
[-0.5, 0.2, 0.8]
```

This confirmed that Python could successfully:

```text
load controller.scs
        ↓
call the C++ SCOTS controller
        ↓
receive a Go2 command
```

---

# 16. Full Python Simulation

A second test was created:

```text
go2_v02/deploy/sim_python.py
```

This reproduced the Go2 dynamics and RK4 integration in Python.

The purpose was to verify the entire path:

```text
Python
   ↓
SCOTS C++ binding
   ↓
controller.scs
   ↓
Go2 command
   ↓
Python simulation
```

The Python simulation reproduced the same 40-step path as the C++ simulator and reached the same target.

This confirmed that the Python/C++ connection was behaving correctly.

---

# 17. Hardware-Independent Deployment Structure

The next step was to separate the control loop from the hardware.

The deployment code now follows this structure:

```text
Pose Source
     ↓
SCOTS Controller
     ↓
Command Sink
```

A pose source answers:

```text
Where is the robot?
```

A command sink answers:

```text
Where should this command be sent?
```

The controller loop does not need to know whether it is running a simulation or controlling the real robot.

---

# 18. PoseSource

The base class is in:

```text
go2_v02/deploy/pose_source.py
```

It defines:

```python
get_pose()
```

which returns:

```text
[x, y, yaw]
```

## SimPoseSource

`SimPoseSource` stores a simulated robot state.

It implements:

```python
get_pose()
```

to return the current state.

It also implements:

```python
apply_control(...)
```

which uses the same Go2 dynamics and RK4 integration as the previous simulator.

This lets the simulated robot move when it receives a controller command.

---

# 19. CommandSink

The base class is in:

```text
go2_v02/deploy/command_sink.py
```

It defines:

```python
send(vx, vy, vyaw)
stop()
```

## SimSink

`SimSink` is used during testing.

When it receives:

```text
[vx, vy, vyaw]
```

it passes the command into:

```python
SimPoseSource.apply_control(...)
```

This makes the simulated robot move.

Later, a real robot sink will use the exact same interface.

---

# 20. Deployment Loop

The main control loop is:

```text
go2_v02/deploy/deploy_loop.py
```

Its basic job is:

```text
read robot pose
       ↓
ask SCOTS for a command
       ↓
send command
       ↓
repeat
```

The loop does not contain Go2-specific hardware code.

This means the same loop can eventually be used with either:

```text
Simulation:
SimPoseSource + SimSink
```

or:

```text
Real robot:
OptiTrackPoseSource + SportClientSink
```

---

# 21. Deployment Safety Behavior

The deployment loop includes an important safety rule.

If SCOTS returns no valid command:

```python
if not controls:
    break
```

the loop stops instead of guessing or reusing the previous command.

This is important because an empty controller result can mean the robot is outside the area where SCOTS has proven a safe route.

The loop also uses:

```python
finally:
    command_sink.stop()
```

so `stop()` is called whenever the loop exits.

That includes:

* reaching the target
* leaving the winning set
* reaching the step limit
* an unexpected error

---

# 22. Full Deployment Dry Run

The new deployment structure was tested inside the deployment Docker container.

From:

```text
go2_v02/synthesis/
```

the test was run with:

```bash
PYTHONPATH=../deploy python3 ../deploy/deploy_loop.py
```

The deployment loop used:

```text
SimPoseSource
+
SCOTS controller
+
SimSink
```

starting from:

```text
[-1.3, 0.0, 0.0]
```

It reproduced the same 40-step serpentine path as both previous simulations.

It ended with:

```text
Reached target.
```

This means three independent paths have now produced the same result:

```text
C++ simulation
      ↓

Python simulation using C++ SCOTS binding
      ↓

Deployment loop using simulated hardware adapters
```

All three reached the same target using the same controller.

---

# What Is Proven So Far

At this point, the project has shown that:

* SCOTS v0.2 builds successfully in Docker.
* The Go2 motion model works with SCOTS v0.2.
* The arena can be configured from a text file.
* SCOTS can generate a winning controller for the current slalom arena.
* The C++ simulator can follow the controller to the target.
* Python can load and query the C++ SCOTS controller.
* The Python simulation matches the C++ simulation.
* The new deployment loop matches both previous simulations.
* The controller outputs Go2 body-frame velocity commands.
* The deployment loop stops if the controller cannot provide a safe command.
* The simulation and future real hardware code are separated cleanly.

---

# What Still Needs to Be Added

The two real hardware pieces are intentionally not implemented yet.

## OptiTrackPoseSource

This will replace:

```text
SimPoseSource
```

on the real system.

It will read the robot pose from the OptiTrack REST endpoint.

The old Go2 code used this frame conversion:

```text
x   = coord[2]
y   = -coord[1]
yaw = coord[3]
```

That conversion needs to stay inside `OptiTrackPoseSource`.

This should be the only place where the OptiTrack coordinate system is converted into the coordinate system expected by SCOTS.

---

## SportClientSink

This will replace:

```text
SimSink
```

on the real robot.

It will use the Unitree Python SDK and eventually call:

```python
SportClient.Move(vx, vy, vyaw)
```

The SCOTS controller already returns body-frame commands, so they should be sent directly to `Move`.

They should not be rotated into the global frame again.

The Unitree SDK import should also happen only when the real sink is created so that the Mac simulation can run without having the Unitree SDK installed.

---

# Planned Final Structure

The goal is for deployment to work like this:

```text
SIM MODE

SimPoseSource
      ↓
SCOTS Controller
      ↓
SimSink
```

and:

```text
REAL MODE

OptiTrackPoseSource
      ↓
SCOTS Controller
      ↓
SportClientSink
```

The controller loop stays the same in both cases.

---

# Current Status

Current progress:

```text
[✓] SCOTS v0.2 Docker environment

[✓] Go2 dynamics

[✓] State and input grids

[✓] Config file parser

[✓] Target definition

[✓] Obstacle definition

[✓] Controller synthesis

[✓] C++ simulation

[✓] Python/C++ SCOTS binding

[✓] Python controller simulation

[✓] Hardware-independent deployment loop

[✓] Simulated PoseSource

[✓] Simulated CommandSink

[✓] Full deployment dry run

[ ] OptiTrackPoseSource

[ ] SportClientSink

[ ] Machine-specific deployment config

[ ] Real Go2 test
```

The important result so far is that the complete controller path has been tested without hardware and produces the same behavior at every stage.
