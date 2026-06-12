/**
 * Robot Control
 */
enum RobotDirection {
    //% block="stop"
    Stop,
    //% block="forward"
    Forward,
    //% block="reverse"
    Reverse,
    //% block="left"
    Left,
    //% block="right"
    Right
}

//% color=#ff7f00 icon="\uf1b9"
//% block="Robot Control"
namespace RobotControl {
    let lastError = 0
    let integral = 0

    /**
     * Calculate PID power difference from an ADC value.
     */
    //% block="PID power diff adc %adc setpoint %setpoint kp %kp kd %kd ki %ki"
    //% adc.min=0 adc.max=1023
    //% setpoint.min=0 setpoint.max=1023
    //% setpoint.defl=512
    //% kp.defl=0.55
    //% kd.defl=0.25
    //% ki.defl=0
    //% inlineInputMode=inline
    export function pid_power_diff(adc: number, setpoint: number, kp: number, kd: number, ki: number): number {
        const error = adc - setpoint
        const derivative = error - lastError

        integral += error
        lastError = error

        return error * kp + derivative * kd + integral * ki
    }

    /**
     * Reset the saved PID values.
     */
    //% block="reset PID"
    export function reset_pid(): void {
        lastError = 0
        integral = 0
    }

    /**
     * Navigate the robot using MOTION:BIT motor channels M4 and M2.
     */
    //% block="navigate %direction speed %speed delay %delay"
    //% speed.min=0 speed.max=255 speed.defl=150
    //% delay.min=0 delay.defl=0
    export function navigate(direction: RobotDirection, speed: number, delay: number): void {
        const motorSpeed = limit(speed, 0, 255)

        if (direction == RobotDirection.Stop) {
            motionbit.brakeMotor(MotionBitMotorChannel.M4)
            motionbit.brakeMotor(MotionBitMotorChannel.M2)
        } else if (direction == RobotDirection.Forward) {
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
     * Enter Maker Line calibration mode using the selected digital pin.
     */
    //% block="enter calibration pin %pin"
    //% pin.defl=DigitalPin.P9
    export function enter_calibration(pin: DigitalPin): void {
        pins.digitalWritePin(pin, 0)
        basic.pause(2100)
        pins.digitalWritePin(pin, 1)
    }

    /**
     * Exit Maker Line calibration mode using the selected digital pin.
     */
    //% block="exit calibration pin %pin"
    //% pin.defl=DigitalPin.P9
    export function exit_calibration(pin: DigitalPin): void {
        pins.digitalWritePin(pin, 0)
        basic.pause(100)
        pins.digitalWritePin(pin, 1)
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
