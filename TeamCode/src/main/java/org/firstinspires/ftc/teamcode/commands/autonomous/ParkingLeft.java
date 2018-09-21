package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.systems.Robot;

/**
 * Created by Mahim on 2/2/18.
 */
@Autonomous(name = "parking left", group = "parking")
public class ParkingLeft extends LinearOpMode {
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
        int count = 0;

        while (opModeIsActive() && (count < 1)) {
            robot.driveSystem.drive(0.5, 0.5);
            sleep(2000);
            count++;
        }
        robot.driveSystem.stop();
    }
}
