package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.CustomOpMode;

@TeleOp(name = "new teleop", group = "testing")
public class TeleOP extends CustomOpMode {
    @Override
    protected void initialize() {
        robot.init(hardwareMap);
    }

    @Override
    protected void execute() {
        // this drives the robot as a mecanum drive.
        robot.drivetrain.drive(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x);

        // this allows the driver to control the arms as they open and close.
        robot.arm.triggerArmServo(gamepad2.right_trigger);

        // if the up arrow is pressed the arm will move up. If it is not
        // pressed it will stop in place.
        if (gamepad2.dpad_up) {
            robot.arm.goUp();
        } else {
            robot.arm.stopArmMotor();
        }

        // if the down arrow is pressed the arm will move down. If it is not
        // pressed it will stop in place.
        if (gamepad2.dpad_down) {
            robot.arm.goDown();
        } else {
            robot.arm.stopArmMotor();
        }

        // if the arm does not stop, pressing the left bumper will hold the
        // arm in position (hopefully.)
        if (gamepad2.left_bumper) {
            robot.arm.stopArmMotor();
        }

        telemetry();
    }

    private void telemetry() {
        robot.drivetrain.getSpeed(telemetry);
        telemetry.addData("left servo", robot.arm.getLeftServoPosition());
        telemetry.addData(
                "right servo", robot.arm.getRightServoPosition());
        telemetry.addData("angle", robot.drivetrain.getAngle());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
