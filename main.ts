/**
 * Robot Control
 */
//% color=#0fbc11 icon="\uf1b9"
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
}
