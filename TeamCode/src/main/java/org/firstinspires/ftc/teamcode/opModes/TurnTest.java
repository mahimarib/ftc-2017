package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by Mahim on 6/16/18.
 */
@Autonomous(name = "Turn Test", group = "Auto")
public class TurnTest extends LinearOpMode {
    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.driveSystem.initEncoders();

        waitForStart();

        robot.driveSystem.turn(90);
        sleep(1000);
        robot.driveSystem.turn(-180);
        sleep(1000);
        robot.driveSystem.turn(90);
        sleep(1000);
        robot.driveSystem.turn(-180);
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}