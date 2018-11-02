package org.firstinspires.ftc.teamcode.commands.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Mahim on 1/3/2018.
 */
@TeleOp(name = "mecanum drive")
public class MecanumDrive extends OpMode {
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        robot.drivetrain.drive(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x);
        robot.armSystem.triggerArmServo(gamepad2.right_trigger);

        if (gamepad2.dpad_up) {
            robot.armSystem.goUp();
        } else {
            robot.armSystem.stopArmMotor();
        }

        if (gamepad2.dpad_down) {
            robot.armSystem.goDown();
        } else {
            robot.armSystem.stopArmMotor();
        }

        if (gamepad2.left_bumper) {
            robot.armSystem.stopArmMotor();
        }

        telemetry();
    }

    private void telemetry() {
        robot.drivetrain.getSpeed(telemetry);
        telemetry.addData("left servo", robot.armSystem.getLeftServoPosition());
        telemetry.addData(
                "right servo", robot.armSystem.getRightServoPosition());
        telemetry.addData("angle", robot.drivetrain.getAngle());
    }

    @Override
    public void stop() {
        robot.drivetrain.stop();
    }
}
