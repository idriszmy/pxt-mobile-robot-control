/**
 * Robot Control
 */
enum RobotDirection {
    //% block="forward"
    Forward,
    //% block="reverse"
    Reverse,
    //% block="left"
    Left,
    //% block="right"
    Right
}

enum TurnDirection {
    //% block="left"
    Left,
    //% block="right"
    Right
}

enum GripperPosition {
    //% block="close"
    Close,
    //% block="open"
    Open,
    //% block="toggle"
    Toggle
}

enum GripperLeftArm {
    //% block="S5"
    S5,
    //% block="S6"
    S6,
    //% block="S7"
    S7,
    //% block="S8"
    S8
}

enum GripperRightArm {
    //% block="S1"
    S1,
    //% block="S2"
    S2,
    //% block="S3"
    S3,
    //% block="S4"
    S4
}

//% color=#3455db icon="\uf1b9"
//% block="Robot Control"
//% groups=["Sensor Calibration", "PID", "Robot", "Gripper"]
namespace RobotControl {
    let lastError = 0
    let integral = 0
    let pidSetpoint = 500
    let pidKp = 0.8
    let pidKd = 0.7
    let pidKi = 0
    let leftMotorChannel = MotionBitMotorChannel.M4
    let rightMotorChannel = MotionBitMotorChannel.M2
    let gripperLeftChannel = MotionBitServoChannel.S8
    let gripperRightChannel = MotionBitServoChannel.S4
    let gripperLeftClosePosition = 90
    let gripperRightClosePosition = 90
    let gripperCurrentPosition = GripperPosition.Open
    let gripperPositionKnown = false
    const gripperRange = 60

    /**
     * Enter Maker Line calibration mode using the selected digital pin.
     */
    //% block="enter calibration pin %pin"
    //% pin.defl=DigitalPin.P9
    //% group="Sensor Calibration"
    //% weight=100
    export function enterCalibration(pin: DigitalPin): void {
        pins.digitalWritePin(pin, 0)
        basic.pause(2100)
        pins.digitalWritePin(pin, 1)
    }

    /**
     * Exit Maker Line calibration mode using the selected digital pin.
     */
    //% block="exit calibration pin %pin"
    //% pin.defl=DigitalPin.P9
    //% group="Sensor Calibration"
    //% weight=90
    export function exitCalibration(pin: DigitalPin): void {
        pins.digitalWritePin(pin, 0)
        basic.pause(100)
        pins.digitalWritePin(pin, 1)
    }

    /**
     * Calculate PID power difference from an ADC value.
     */
    //% block="PID power diff adc %adc"
    //% adc.min=0 adc.max=1023
    //% group="PID"
    //% weight=80
    export function pidPowerDiff(adc: number): number {
        const error = adc - pidSetpoint
        const derivative = error - lastError

        integral += error
        lastError = error

        return error * pidKp + derivative * pidKd + integral * pidKi
    }

    /**
     * Set the PID tuning values.
     */
    //% block="set PID tuning setpoint %setpoint kp %kp kd %kd ki %ki"
    //% setpoint.min=0 setpoint.max=1023 setpoint.defl=500
    //% kp.defl=0.8
    //% kd.defl=0.7
    //% ki.defl=0
    //% inlineInputMode=inline
    //% group="PID"
    //% weight=100
    export function setPidPowerDiff(setpoint: number, kp: number, kd: number, ki: number): void {
        pidSetpoint = limit(setpoint, 0, 1023)
        pidKp = kp
        pidKd = kd
        pidKi = ki
        resetPid()
    }

    /**
     * Set the left and right motor channels.
     */
    //% block="set motor left %leftChannel right %rightChannel"
    //% leftChannel.defl=MotionBitMotorChannel.M4
    //% rightChannel.defl=MotionBitMotorChannel.M2
    //% group="Robot"
    //% weight=110
    export function setMotor(leftChannel: MotionBitMotorChannel, rightChannel: MotionBitMotorChannel): void {
        leftMotorChannel = leftChannel
        rightMotorChannel = rightChannel
    }

    /**
     * Navigate the robot using MOTION:BIT motor channels M4 and M2.
     */
    //% block="robot navigate %direction speed %speed delay %delay"
    //% speed.min=0 speed.max=255 speed.defl=170
    //% delay.min=0 delay.defl=100
    //% group="Robot"
    //% weight=90
    export function robotNavigate(direction: RobotDirection, speed: number, delay: number): void {
        const motorSpeed = limit(speed, 0, 255)

        if (direction == RobotDirection.Forward) {
            motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
            motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
        } else if (direction == RobotDirection.Reverse) {
            motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
            motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
        } else if (direction == RobotDirection.Right) {
            motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
            motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
        } else if (direction == RobotDirection.Left) {
            motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
            motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
        }

        if (delay > 0) {
            basic.pause(delay)
            robotStop()
        }
    }

    /**
     * Stop the robot using MOTION:BIT motor channels M4 and M2.
     */
    //% block="robot stop"
    //% group="Robot"
    //% weight=60
    export function robotStop(): void {
        motionbit.brakeMotor(leftMotorChannel)
        motionbit.brakeMotor(rightMotorChannel)
    }

    /**
     * Follow a line using Maker Line ADC input and MOTION:BIT motors.
     */
    //% block="robot line follow pin %pin speed %speed cross %cross timer to stop %stopTimer"
    //% pin.defl=AnalogReadWritePin.P0
    //% speed.min=0 speed.max=255 speed.defl=220
    //% cross.shadow="toggleOnOff"
    //% cross.defl=true
    //% stopTimer.min=0 stopTimer.defl=0
    //% inlineInputMode=inline
    //% group="Robot"
    //% weight=80
    export function robotLineFollow(pin: AnalogReadWritePin, speed: number, cross: boolean, stopTimer: number): void {
        const baseSpeed = limit(speed, 0, 255)
        let speedLeft = baseSpeed
        let speedRight = baseSpeed
        let crossFound = false
        let endTime = 0
        let timerEndTime = 0

        resetPid()

        if (!cross && stopTimer > 0) {
            timerEndTime = input.runningTime() + stopTimer
        }

        while (true) {
            const adc = pins.analogReadPin(pin)

            if (!cross && timerEndTime > 0 && input.runningTime() >= timerEndTime) {
                break
            }

            if (adc > 941 && cross) {
                if (stopTimer <= 0) {
                    break
                }
                if (!crossFound) {
                    crossFound = true
                    endTime = input.runningTime() + stopTimer
                }
            }

            if (crossFound && input.runningTime() >= endTime) {
                break
            }

            if (adc < 81) {
                if (lastError < 0) {
                    speedLeft = 0
                    speedRight = baseSpeed
                } else {
                    speedLeft = baseSpeed
                    speedRight = 0
                }
                runLineMotors(speedLeft, speedRight)
            } else if (adc > 941) {
                speedLeft = baseSpeed
                speedRight = baseSpeed
                runLineMotors(speedLeft, speedRight)
            } else {
                const powerDiff = limit(pidPowerDiff(adc), -baseSpeed, baseSpeed)

                if (powerDiff < 0) {
                    speedLeft = baseSpeed + powerDiff
                    speedRight = baseSpeed
                } else {
                    speedLeft = baseSpeed
                    speedRight = baseSpeed - powerDiff
                }

                runLineMotors(speedLeft, speedRight)
            }

            basic.pause(5)
        }

        robotStop()
    }

    /**
     * Calibrate Maker Line by rotating the robot using MOTION:BIT motors.
     */
    //% block="robot calibration pin %pin speed %speed"
    //% pin.defl=DigitalPin.P9
    //% speed.min=0 speed.max=255 speed.defl=170
    //% group="Robot"
    //% weight=100
    export function robotCalibration(pin: DigitalPin, speed: number): void {
        const motorSpeed = limit(speed, 0, 255)

        enterCalibration(pin)
        motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
        motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
        basic.pause(1000)
        motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
        motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
        basic.pause(2000)
        motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
        motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
        basic.pause(1000)
        robotStop()
        exitCalibration(pin)
    }

    /**
     * Turn the robot until it finds the line again.
     */
    //% block="robot turn to line %direction speed %speed pin %pin"
    //% speed.min=0 speed.max=255 speed.defl=170
    //% pin.defl=AnalogReadWritePin.P0
    //% inlineInputMode=inline
    //% group="Robot"
    //% weight=70
    export function robotTurnToLine(direction: TurnDirection, speed: number, pin: AnalogReadWritePin): void {
        const motorSpeed = limit(speed, 0, 255)

        if (direction == TurnDirection.Left) {
            motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
            motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
        } else if (direction == TurnDirection.Right) {
            motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Forward, motorSpeed)
            motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Backward, motorSpeed)
        }

        while (pins.analogReadPin(pin) >= 81) {
            basic.pause(5)
        }

        basic.pause(200)

        while (pins.analogReadPin(pin) < 81) {
            basic.pause(5)
        }

        robotStop()
    }

    /**
     * Save the gripper close positions.
     */
    //% block="gripper close alignment left arm %leftArm align %leftAlignment right arm %rightArm align %rightAlignment"
    //% leftArm.defl=GripperLeftArm.S8
    //% leftAlignment.min=-20 leftAlignment.max=20 leftAlignment.defl=0
    //% rightArm.defl=GripperRightArm.S4
    //% rightAlignment.min=-20 rightAlignment.max=20 rightAlignment.defl=0
    //% inlineInputMode=inline
    //% group="Gripper"
    //% weight=100
    export function calibrateGripperClose(leftArm: GripperLeftArm, leftAlignment: number, rightArm: GripperRightArm, rightAlignment: number): void {
        gripperLeftChannel = gripperLeftArmChannel(leftArm)
        gripperRightChannel = gripperRightArmChannel(rightArm)
        gripperLeftClosePosition = invertServoPosition(alignmentPosition(leftAlignment))
        gripperRightClosePosition = invertServoPosition(alignmentPosition(rightAlignment))
        gripperPositionKnown = false
    }

    /**
     * Move the gripper to open or close.
     */
    //% block="gripper %position"
    //% group="Gripper"
    //% weight=90
    export function gripper(position: GripperPosition): void {
        if (position == GripperPosition.Toggle) {
            if (!gripperPositionKnown || gripperCurrentPosition == GripperPosition.Open) {
                position = GripperPosition.Close
            } else {
                position = GripperPosition.Open
            }
        }

        if (gripperPositionKnown && position == gripperCurrentPosition) {
            return
        }

        if (position == GripperPosition.Close) {
            moveGripper(
                gripperLeftClosePosition + gripperRange,
                gripperLeftClosePosition,
                gripperRightClosePosition - gripperRange,
                gripperRightClosePosition
            )
        } else {
            moveGripper(
                gripperLeftClosePosition,
                gripperLeftClosePosition + gripperRange,
                gripperRightClosePosition,
                gripperRightClosePosition - gripperRange
            )
        }

        gripperCurrentPosition = position
        gripperPositionKnown = true
    }

    function runLineMotors(speedLeft: number, speedRight: number): void {
        motionbit.runMotor(leftMotorChannel, MotionBitMotorDirection.Forward, limit(speedLeft, 0, 255))
        motionbit.runMotor(rightMotorChannel, MotionBitMotorDirection.Forward, limit(speedRight, 0, 255))
    }

    function moveGripper(leftFrom: number, leftTo: number, rightFrom: number, rightTo: number): void {
        for (let step = 0; step <= gripperRange; step++) {
            const leftPosition = leftFrom + (leftTo - leftFrom) * step / gripperRange
            const rightPosition = rightFrom + (rightTo - rightFrom) * step / gripperRange

            motionbit.setServoPosition(gripperLeftChannel, limit(leftPosition, 0, 180))
            motionbit.setServoPosition(gripperRightChannel, limit(rightPosition, 0, 180))
            basic.pause(5)
        }

        basic.pause(200)
    }

    function invertServoPosition(position: number): number {
        return 180 - limit(position, 0, 180)
    }

    function alignmentPosition(alignment: number): number {
        return 90 + limit(alignment, -20, 20)
    }

    function gripperLeftArmChannel(arm: GripperLeftArm): MotionBitServoChannel {
        if (arm == GripperLeftArm.S5) {
            return MotionBitServoChannel.S5
        } else if (arm == GripperLeftArm.S6) {
            return MotionBitServoChannel.S6
        } else if (arm == GripperLeftArm.S7) {
            return MotionBitServoChannel.S7
        } else {
            return MotionBitServoChannel.S8
        }
    }

    function gripperRightArmChannel(arm: GripperRightArm): MotionBitServoChannel {
        if (arm == GripperRightArm.S1) {
            return MotionBitServoChannel.S1
        } else if (arm == GripperRightArm.S2) {
            return MotionBitServoChannel.S2
        } else if (arm == GripperRightArm.S3) {
            return MotionBitServoChannel.S3
        } else {
            return MotionBitServoChannel.S4
        }
    }

    function resetPid(): void {
        lastError = 0
        integral = 0
    }

    function limit(value: number, min: number, max: number): number {
        if (value < min) {
            return min
        }
        if (value > max) {
            return max
        }
        return value
    }
}
