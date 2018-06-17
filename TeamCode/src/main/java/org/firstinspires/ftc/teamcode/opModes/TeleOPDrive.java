package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by Mahim on 6/16/18.
 */
@TeleOp(name = "teleOP")
public class TeleOPDrive extends OpMode {
    private Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.driveTrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        robot.driveTrain.getSpeed(telemetry);
    }
}
