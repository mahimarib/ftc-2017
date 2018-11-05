package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.CustomOpMode;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;

@Autonomous(name = "new blue alliance bottom", group = "testing")
public class BlueBottom extends CustomOpMode {
    @Override
    public void initialize() {
        robot.init(hardwareMap);
    }

    @Override
    public void execute() {
        robot.knockDownRedJewel();
        robot.drivetrain.stop();
        sleep(1000);
        robot.drivetrain.drive(0.5, 0.3, Drivetrain.Direction.FORWARD);
        sleep(1000);
        robot.drivetrain.stop();
    }
}
