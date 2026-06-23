# Mobile Robot Control

Microsoft MakeCode extension for micro:bit mobile robot activities using MOTION:BIT.

![Mobile Robot Control](https://raw.githubusercontent.com/idriszmy/pxt-mobile-robot-control/master/icon.png)

This extension provides higher-level robot blocks for line following, PID tuning, Maker Line calibration, robot movement, and gripper servo alignment. It is designed for classroom robotics challenges where students need reliable blocks without rebuilding the same control logic for every project.

## Features

* PID power difference calculator with editable tuning values
* Maker Line sensor calibration blocks
* Line following with cross detection and timer-to-stop support
* Robot movement blocks using configurable left and right motor channels
* Turn-to-line helper for left and right turns
* Gripper close alignment and open/close/toggle control

## Hardware

This extension is intended for:

* BBC micro:bit
* MOTION:BIT
* Maker Line or compatible analog line sensor
* DC motors connected to MOTION:BIT motor channels
* Optional gripper servos connected to MOTION:BIT servo channels

Default setup:

* Left motor: M4
* Right motor: M2
* Line sensor: P0
* Maker Line calibration pin: P9
* Left gripper arm: S8
* Right gripper arm: S4

These defaults can be changed using the provided setup blocks.

## Blocks

### Sensor Calibration

* `enter calibration pin`
* `exit calibration pin`

### PID

* `set PID tuning setpoint kp kd ki`
* `PID power diff adc`

Default PID values:

* setpoint: `500`
* kp: `0.8`
* kd: `0.7`
* ki: `0`

### Robot

* `set motor left right`
* `robot calibration`
* `robot navigate`
* `robot line follow`
* `robot turn to line`
* `robot stop`

### Gripper

* `gripper close alignment`
* `gripper close/open/toggle`

## Use as Extension

This repository can be added as an extension in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click **New Project**
* click **Extensions** under the gearwheel menu
* search for `https://github.com/idriszmy/pxt-mobile-robot-control` and import

## Example

```typescript
RobotControl.setMotor(MotionBitMotorChannel.M4, MotionBitMotorChannel.M2)
RobotControl.setPidPowerDiff(500, 0.8, 0.7, 0)
RobotControl.robotLineFollow(AnalogReadWritePin.P0, 220, true, 0)
RobotControl.robotTurnToLine(TurnDirection.Left, 170, AnalogReadWritePin.P0)
```

## Edit this project

To edit this repository in MakeCode:

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click **Import**
* click **Import URL**
* paste `https://github.com/idriszmy/pxt-mobile-robot-control`
* click import

## License

MIT

#### Metadata (used for search, rendering)

* for PXT/microbit
<script src="https://makecode.com/gh-pages-embed.js"></script><script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>
