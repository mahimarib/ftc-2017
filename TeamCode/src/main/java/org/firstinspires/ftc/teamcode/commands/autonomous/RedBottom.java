package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Mahim on 1/13/2018.
 */
@Autonomous(name = "red alliance bottom", group = "red alliance")
public class RedBottom extends LinearOpMode {
    private Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    private void initialize() {
        robot.init(hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        runtime.reset();
        int count = 0;

        while (opModeIsActive() && (count < 1)) {
            robot.knockDownBlueJewel();
            robot.driveSystem.stop();
            sleep(1000);
            robot.driveSystem.drive(0.5, 0.5, MecanumDriveSystem.Direction.REVERSE);
            sleep(1000);
            robot.driveSystem.stop();
            count++;
        }
    }
}