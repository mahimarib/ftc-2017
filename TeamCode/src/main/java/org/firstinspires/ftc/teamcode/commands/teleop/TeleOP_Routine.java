package org.firstinspires.ftc.teamcode.commands.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * This is the tele-operated routine for the robot. This routine requires two
 * drivers.
 */
@TeleOp(name = "TeleOP")
public class TeleOP_Routine extends OpMode {
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This method runs before the routine starts. This initializes the robot.
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {}

    /**
     * This method runs when the opMode first starts
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * This is the main loop, this is played through the match.
     */
    @Override
    public void loop() {
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

        // prints out all hardware values to test.
        telemetry();
    }

    /**
     * This method contains all the values you want to check to test new
     * actions.
     */
    private void telemetry() {
        robot.drivetrain.getSpeed(telemetry);
        telemetry.addData("left servo", robot.arm.getLeftServoPosition());
        telemetry.addData(
                "right servo", robot.arm.getRightServoPosition());
        telemetry.addData("angle", robot.drivetrain.getAngle());
    }

    /**
     * This method runs when the robot is pressed stop.
     */
    @Override
    public void stop() {
        robot.drivetrain.stop();
    }
}
