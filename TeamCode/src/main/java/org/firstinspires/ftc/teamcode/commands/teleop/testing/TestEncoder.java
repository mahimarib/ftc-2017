package org.firstinspires.ftc.teamcode.commands.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.systems.DriveSystem.Direction;

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
            robot.driveSystem.drive(0.5, 0.5, Direction.FORWARD);
        } else {
            robot.driveSystem.stop();
        }
        telemetry();
    }

    private void telemetry() {}

    @Override
    public void stop() {
        robot.driveSystem.stop();
    }
}
