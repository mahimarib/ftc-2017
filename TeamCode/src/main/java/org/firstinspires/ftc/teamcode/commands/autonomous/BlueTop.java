package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.systems.Drivetrain.Direction;

/**
 * This Autonomous routine is when the robot is on the blue alliance and on
 * the balancing scale that is furthest to the relic mat.
 * <p>
 * This routine knocks down the jewel and drives into the safe zone.
 */
@Autonomous(name = "blue alliance top", group = "blue alliance")
public class BlueTop extends LinearOpMode {
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
        robot.knockDownRedJewel();
        robot.drivetrain.stop();
        sleep(1000);
        robot.drivetrain.drive(0.5, 0.3, Direction.FORWARD);
        sleep(1000);
        robot.drivetrain.stop();
    }
}