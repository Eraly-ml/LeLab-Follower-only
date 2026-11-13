# LeLab Follower-only

Control an SO-101 follower arm from a browser without a physical leader arm. This is an experimental modification of [LeLab](https://github.com/huggingface/leLab), built on [LeRobot](https://github.com/huggingface/lerobot).

## Why this project exists

A typical SO-101 teleoperation setup uses two arms: a **leader** that a person moves and a **follower** that copies it. We wanted to work with a single follower, so we added a virtual control mode to LeLab. The follower is still a real robot; the virtual part is the input normally supplied by the leader.

This makes it possible to explore and manually control a calibrated arm without buying or connecting a second arm. It does not turn mouse control into a physical leader, and the whole-arm control is still experimental.

## What we added

- **Follower-only teleoperation:** start a session with a configured, calibrated SO-101 follower and no leader configuration.
- **3D joint control:** select **Суставы** (Joints), then drag an individual part of the displayed robot with the mouse.
- **Whole-arm control:** select **Вся рука** (Whole arm) and drag the model to move the gripper position in the plane of the screen. Inverse kinematics calculates joint targets from that movement.
- **Keyboard gripper control:** hold **W** to close and **E** to open the gripper. Release the key to stop the command.
- **Motion limits:** commands use the saved follower calibration, bounded target offsets and gradual changes. Closing reaches the calibrated closed limit.

The whole-arm solver controls the gripper's **position**, not its orientation. It chooses which joints to move, so not every motor necessarily moves with every drag. One drag is limited to 6 cm; release and drag again to continue. Joint and calibration limits can prevent a movement even when the pointer keeps moving.

## Requirements

- An SO-101 follower, connected by USB and powered by its own power supply.
- A follower calibrated with LeRobot or LeLab. A leader arm or leader calibration is not required for this mode.
- Python 3.12 or newer, Git and a browser. On Linux, the user needs permission to access the robot's serial port.
- A source checkout: the experimental kinematics solver currently loads its robot model from the source tree.

## Installation

Clone this repository and create a virtual environment:

```bash
git clone https://github.com/Eraly-ml/LeLab-Follower-only.git
cd LeLab-Follower-only
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
python -m pip install 'placo>=0.9.6,<0.9.16' 'cmeel-urdfdom>=4,<5' 'cmeel-tinyxml2<11'
```

The repository is private unless its owner changes its visibility, so cloning requires access. These commands are for a Unix-like shell; activate the virtual environment using the appropriate command on Windows.

Build the frontend once for the regular application server (Node.js and npm are required):

```bash
cd frontend
npm ci
npm run build
cd ..
lelab
```

Open the address printed by LeLab, normally [http://localhost:8000](http://localhost:8000).

## Getting started

1. Secure the follower, connect USB and power, and make sure its workspace is clear.
2. In LeLab, configure the follower's serial port and calibration. Calibrate the follower if needed. Leave the leader configuration empty for follower-only mode.
3. On the robot card, start **Teleoperation**. A configured follower without a leader is shown as **Virtual preview**.
4. Choose **Суставы** for individual joint control or **Вся рука** for whole-arm movement. Start with small, slow movements.
5. Hold **W** to close the gripper and **E** to open it. Keep the browser window focused when using the keyboard.

The first whole-arm interaction can take longer while the kinematics solver initializes. If motion is restricted, check the arm's pose and calibration: a joint may be near its configured limit or the requested position may be unreachable.

## Safety and current limitations

This is experimental software tested with one SO-101 follower. Calibration, hardware, motor strength and mechanical assembly can differ. The robot can move when you interact with the controls.

- There is **no collision avoidance** or obstacle detection. Keep people, cables and objects clear of the robot's workspace.
- Support the arm when stopping the session, closing the page or losing the connection: motors may release and the arm may drop.
- Stop if the arm binds, hits a limit or a motor strains. Do not force the gripper against an obstruction.
- The 3D model is a control and visualization aid; it is not a guarantee of the robot's exact physical pose or clearance.
- Follower-only mouse control is for manual teleoperation. Other inherited LeLab workflows have not all been verified with this mode.

## How it works

The browser displays the SO-101 URDF model and sends joint or Cartesian targets to a local LeLab server. The server reads follower positions through LeRobot and checks targets against calibration and movement limits. For whole-arm dragging, LeRobot's kinematics interface and Placo solve for joint positions. On release, the control session requests the current position as a hold target; on disconnection or loss of the page, the session stops.

## Upstream and license

This project builds on [LeLab](https://github.com/huggingface/leLab) and [LeRobot](https://github.com/huggingface/lerobot). For the original LeLab's other features and documentation, see the upstream repository.

Licensed under the [Apache License 2.0](LICENSE).
