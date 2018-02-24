package org.firstinspires.ftc.teamcode.commands.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;

/**
 * Created by nycfirst on 2/18/18.
 */
@Autonomous(name = "turn command", group = "test")
public class TurnCommand extends OpMode {
    private MecanumDriveSystem mecanumDriveSystem;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        mecanumDriveSystem = new MecanumDriveSystem(hardwareMap);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        mecanumDriveSystem.setAngle(5.0);
        telemetry.addData("angle", mecanumDriveSystem.getAngle());
    }

    @Override
    public void stop() {
        mecanumDriveSystem.stop();
    }
}
