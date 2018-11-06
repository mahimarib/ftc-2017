package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.CustomOpMode;

/**
 * This opMode is used to test the turn command of the drivetrain.
 */
@Autonomous(name = "turn test", group = "testing")
public class TurnTest extends CustomOpMode {
    @Override
    public void initialize() {
        robot.init(hardwareMap);
    }

    @Override
    public void execute() {
        robot.drivetrain.turn(90);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
