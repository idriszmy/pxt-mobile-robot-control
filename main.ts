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

enum TurnAngle {
    //% block="90"
    Angle90 = 90,
    //% block="180"
    Angle180 = 180
}

//% color=#3455db icon="\uf1b9"
//% block="Robot Control"
//% groups=["Sensor Calibration", "PID", "Robot"]
namespace RobotControl {
    let lastError = 0
    let integral = 0
    let pidSetpoint = 512
    let pidKp = 1.2
    let pidKd = 0.8
    let pidKi = 0

    /**
     * Enter Maker Line calibration mode using the selected digital pin.
     */
    //% block="enter calibration pin %pin"
    //% pin.defl=DigitalPin.P9
    //% group="Sensor Calibration"
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
    //% setpoint.min=0 setpoint.max=1023 setpoint.defl=512
    //% kp.defl=1.2
    //% kd.defl=0.8
    //% ki.defl=0
    //% inlineInputMode=inline
    //% group="PID"
    export function setPidPowerDiff(setpoint: number, kp: number, kd: number, ki: number): void {
        pidSetpoint = limit(setpoint, 0, 1023)
        pidKp = kp
        pidKd = kd
        pidKi = ki
        resetPid()
    }

    /**
     * Reset the saved PID values.
     */
    //% block="reset PID"
    //% group="PID"
    export function resetPid(): void {
        lastError = 0
        integral = 0
    }

    /**
     * Navigate the robot using MOTION:BIT motor channels M4 and M2.
     */
    //% block="robot navigate %direction speed %speed delay %delay"
    //% speed.min=0 speed.max=255 speed.defl=170
    //% delay.min=0 delay.defl=0
    //% group="Robot"
    export function robotNavigate(direction: RobotDirection, speed: number, delay: number): void {
        const motorSpeed = limit(speed, 0, 255)

        if (direction == RobotDirection.Forward) {
            motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Forward, motorSpeed)
            motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Forward, motorSpeed)
        } else if (direction == RobotDirection.Reverse) {
            motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Backward, motorSpeed)
            motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Backward, motorSpeed)
        } else if (direction == RobotDirection.Right) {
            motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Forward, motorSpeed)
            motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Backward, motorSpeed)
        } else if (direction == RobotDirection.Left) {
            motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Backward, motorSpeed)
            motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Forward, motorSpeed)
        }

        basic.pause(Math.max(0, delay))
    }

    /**
     * Stop the robot using MOTION:BIT motor channels M4 and M2.
     */
    //% block="robot stop"
    //% group="Robot"
    export function robotStop(): void {
        motionbit.brakeMotor(MotionBitMotorChannel.M4)
        motionbit.brakeMotor(MotionBitMotorChannel.M2)
    }

    /**
     * Follow a line using Maker Line ADC input and MOTION:BIT motors.
     */
    //% block="robot line follow pin %pin speed %speed cross %cross timer %timer"
    //% pin.defl=AnalogReadWritePin.P0
    //% speed.min=0 speed.max=255 speed.defl=220
    //% cross.shadow="toggleOnOff"
    //% cross.defl=true
    //% timer.min=0 timer.defl=0
    //% inlineInputMode=inline
    //% group="Robot"
    export function robotLineFollow(pin: AnalogReadWritePin, speed: number, cross: boolean, timer: number): void {
        const baseSpeed = limit(speed, 0, 255)
        let speedLeft = baseSpeed
        let speedRight = baseSpeed
        let crossFound = false
        let endTime = 0

        resetPid()

        while (true) {
            const adc = pins.analogReadPin(pin)

            if (adc > 941 && cross) {
                if (timer <= 0) {
                    break
                }
                if (!crossFound) {
                    crossFound = true
                    endTime = input.runningTime() + timer
                }
            }

            if (crossFound && input.runningTime() >= endTime) {
                break
            }

            if (adc < 81) {
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
    }

    /**
     * Calibrate Maker Line by rotating the robot using MOTION:BIT motors.
     */
    //% block="robot calibration pin %pin speed %speed"
    //% pin.defl=DigitalPin.P9
    //% speed.min=0 speed.max=255 speed.defl=170
    //% group="Robot"
    export function robotCalibration(pin: DigitalPin, speed: number): void {
        const motorSpeed = limit(speed, 0, 255)

        enterCalibration(pin)
        motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Backward, motorSpeed)
        motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Forward, motorSpeed)
        basic.pause(1000)
        motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Forward, motorSpeed)
        motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Backward, motorSpeed)
        basic.pause(2000)
        motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Backward, motorSpeed)
        motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Forward, motorSpeed)
        basic.pause(1000)
        motionbit.brakeMotor(MotionBitMotorChannel.All)
        exitCalibration(pin)
    }

    /**
     * Turn the robot until it finds the line again.
     */
    //% block="robot turn to line %direction speed %speed angle %angle pin %pin"
    //% speed.min=0 speed.max=255 speed.defl=170
    //% angle.defl=TurnAngle.Angle90
    //% pin.defl=AnalogReadWritePin.P0
    //% inlineInputMode=inline
    //% group="Robot"
    export function robotTurnToLine(direction: TurnDirection, speed: number, angle: TurnAngle, pin: AnalogReadWritePin): void {
        const motorSpeed = limit(speed, 0, 255)

        if (direction == TurnDirection.Left) {
            motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Backward, motorSpeed)
            motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Forward, motorSpeed)
        } else if (direction == TurnDirection.Right) {
            motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Forward, motorSpeed)
            motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Backward, motorSpeed)
        }

        if (angle == TurnAngle.Angle90) {
            basic.pause(300)
        } else if (angle == TurnAngle.Angle180) {
            basic.pause(600)
        }

        while (pins.analogReadPin(pin) < 81) {
            basic.pause(5)
        }
    }

    function runLineMotors(speedLeft: number, speedRight: number): void {
        motionbit.runMotor(MotionBitMotorChannel.M4, MotionBitMotorDirection.Forward, limit(speedLeft, 0, 255))
        motionbit.runMotor(MotionBitMotorChannel.M2, MotionBitMotorDirection.Forward, limit(speedRight, 0, 255))
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
