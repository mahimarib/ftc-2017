package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by nycfirst on 6/20/18.
 */
@Autonomous(name = "Drive distance")
public class DriveDistanceTest extends LinearOpMode {
    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.driveSystem.initEncoders();

        waitForStart();

        robot.driveSystem.driveDistance(55);
        sleep(1000);

        robot.driveSystem.turn(-90);
        sleep(1000);

        robot.driveSystem.driveDistance(77.5);
        sleep(1000);
    }
}