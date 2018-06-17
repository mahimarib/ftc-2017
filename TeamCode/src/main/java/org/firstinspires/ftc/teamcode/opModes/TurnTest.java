package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by Mahim on 6/16/18.
 */
@Autonomous(name = "Turn Test")
public class TurnTest extends LinearOpMode {
    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot
        robot.init(hardwareMap);
        robot.driveTrain.initEncoders();

        // Wait until we're told to go
        waitForStart();

        robot.driveTrain.turn(-90, 5.0);

        telemetry.addData("Path", "Complete");
        robot.driveTrain.getSpeed(telemetry);
        telemetry.update();
    }
}
