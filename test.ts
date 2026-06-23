// Compile smoke tests for the public API.
// This file is not compiled when the package is used as an extension.

if (false) {
    RobotControl.enterCalibration(DigitalPin.P9)
    RobotControl.exitCalibration(DigitalPin.P9)

    RobotControl.setPidPowerDiff(500, 0.8, 0.7, 0)
    RobotControl.pidPowerDiff(512)

    RobotControl.setMotor(MotionBitMotorChannel.M4, MotionBitMotorChannel.M2)
    RobotControl.robotCalibration(DigitalPin.P9, 170)
    RobotControl.robotNavigate(RobotDirection.Forward, 170, 100)
    RobotControl.robotLineFollow(AnalogReadWritePin.P0, 220, true, 0)
    RobotControl.robotTurnToLine(TurnDirection.Left, 170, AnalogReadWritePin.P0)
    RobotControl.robotStop()

    RobotControl.calibrateGripperClose(GripperLeftArm.S8, 0, GripperRightArm.S4, 0)
    RobotControl.gripper(GripperPosition.Toggle)
}
