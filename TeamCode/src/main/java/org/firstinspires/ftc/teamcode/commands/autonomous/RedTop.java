package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.systems.Drivetrain.Direction;

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
        robot.knockDownBlueJewel();
        robot.drivetrain.stop();
        sleep(1000);
        robot.drivetrain.drive(0.3, 0.5, Direction.FORWARD);
        sleep(1000);
        robot.drivetrain.stop();
    }
}