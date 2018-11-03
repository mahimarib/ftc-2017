package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.systems.Drivetrain.Direction;

/**
 * This Autonomous routine is when the robot is on the red alliance and on
 * the balancing scale that is closest to the relic mat.
 * <p>
 * This routine knocks down the jewel and drives into the safe zone.
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
        robot.knockDownBlueJewel();
        robot.drivetrain.stop();
        sleep(1000);
        robot.drivetrain.drive(0.5, 0.5, Direction.REVERSE);
        sleep(1000);
        robot.drivetrain.stop();
    }
}