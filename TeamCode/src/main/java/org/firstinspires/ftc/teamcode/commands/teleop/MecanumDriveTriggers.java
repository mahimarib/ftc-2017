package org.firstinspires.ftc.teamcode.commands.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;

/**
 * Created by nycfirst on 2/23/18.
 */
@Disabled
@TeleOp(name = "mecanum drive triggers")
public class MecanumDriveTriggers extends OpMode {
    private MecanumDriveSystem mecanumDriveSystem;
    private ArmSystem armSystem;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        this.armSystem = new ArmSystem(hardwareMap);
        this.mecanumDriveSystem = new MecanumDriveSystem(hardwareMap);
    }

    @Override
    public void start() { runtime.reset(); }

    @Override
    public void loop() {
        this.mecanumDriveSystem.mecanumDrive(gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.left_stick_y, gamepad1.right_stick_y);
        this.armSystem.triggerArmServo(gamepad2.right_trigger);

        if(gamepad2.dpad_up) {
            armSystem.goUp();
        } else {
            armSystem.stopArmMotor();
        }

        if(gamepad2.dpad_down) {
            armSystem.goDown();
        } else {
            armSystem.stopArmMotor();
        }

        if(gamepad2.left_bumper) {
            armSystem.stopArmMotor();
        }

        telemetry();
    }

    private void telemetry() {
        telemetry.addData("front left speed", mecanumDriveSystem.getFrontLeftSpeed());
        telemetry.addData("rear left speed", mecanumDriveSystem.getRearLeftSpeed());
        telemetry.addData("front right speed", mecanumDriveSystem.getFrontRightSpeed());
        telemetry.addData("rear right speed", mecanumDriveSystem.getRearRightSpeed());
        telemetry.addData("left servo", armSystem.getLeftServoPosition());
        telemetry.addData("right servo", armSystem.getRightServoPosition());
        telemetry.addData("angle", mecanumDriveSystem.getAngle());
    }

    @Override
    public void stop() {
        this.mecanumDriveSystem.stop();
    }
}
