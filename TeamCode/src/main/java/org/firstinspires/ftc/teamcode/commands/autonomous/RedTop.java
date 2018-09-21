package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.Robot;

/**
 * Created by Mahim on 1/12/2018.
 */
@Autonomous(name = "red alliance top", group = "red alliance")
public class RedTop extends LinearOpMode {
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
            robot.knockDownJewel(Robot.Jewels.BLUE);
            robot.driveSystem.stop();
            sleep(1000);
            robot.driveSystem.drive(0.3, 0.5, MecanumDriveSystem.Direction.FORWARD);
            sleep(1000);
            robot.driveSystem.stop();
            count++;
        }
    }
}