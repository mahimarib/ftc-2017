package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * This opMode is used to test the turn command of the drivetrain.
 */
@Autonomous(name = "turn test", group = "test")
public class TurnTest extends LinearOpMode {
    private Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.drivetrain.turn(90);
        robot.drivetrain.stop();
    }
}
