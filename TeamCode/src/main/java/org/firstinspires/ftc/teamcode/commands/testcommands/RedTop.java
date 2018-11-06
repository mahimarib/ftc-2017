package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.CustomOpMode;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;

@Autonomous(name = "new red alliance top", group = "testing")
public class RedTop extends CustomOpMode {
    @Override
    public void initialize() {
        robot.init(hardwareMap);
    }

    @Override
    public void execute() {
        robot.knockDownBlueJewel();
        robot.drivetrain.stop();
        sleep(1000);
        robot.drivetrain.drive(0.3, 0.5, Drivetrain.Direction.FORWARD);
        sleep(1000);
        robot.drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
