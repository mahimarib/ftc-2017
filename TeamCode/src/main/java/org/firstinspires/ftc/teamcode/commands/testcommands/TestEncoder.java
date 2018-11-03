package org.firstinspires.ftc.teamcode.commands.testcommands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.systems.Drivetrain.Direction;

/**
 * Created by Mahim on 1/9/18.
 */
@TeleOp(name = "testing encoders", group = "testing")
public class TestEncoder extends OpMode {
    private Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.drivetrain.drive(0.5, 0.5, Direction.FORWARD);
        } else {
            robot.drivetrain.stop();
        }
        telemetry();
    }

    private void telemetry() {}

    @Override
    public void stop() {
        robot.drivetrain.stop();
    }
}
