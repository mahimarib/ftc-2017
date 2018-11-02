package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.systems.DriveSystem.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.systems.DriveSystem.Direction.REVERSE;

/**
 * Created by Mahim on 10/31/2018.
 */
@Autonomous (name = "gala autonomous", group = "Test")
public class RelicRed extends LinearOpMode {
    private Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.armSystem.triggerArmServo(0);
        sleep(1000);
        robot.knockDownBlueJewel();
        robot.driveSystem.drive(.5, .5, FORWARD);
        sleep(2500);
        robot.driveSystem.drive(.3, .5, FORWARD);
        robot.armSystem.triggerArmServo(1);
        robot.driveSystem.drive(.5, .3, REVERSE);
        robot.driveSystem.drive(.5, .5, REVERSE);

    }
}
