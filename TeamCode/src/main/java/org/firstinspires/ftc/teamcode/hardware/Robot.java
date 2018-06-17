package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.systems.template.Mechanism;

/**
 * Created by Mahim on 6/16/18.
 */

public class Robot extends Mechanism {
    public DriveTrain driveTrain;

    public Robot() {
        driveTrain = new DriveTrain();
    }

    public Robot(LinearOpMode opMode) {
        driveTrain = new DriveTrain(opMode);
    }

    @Override
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
    }

    public void waitForStart() {
        while (!opMode.isStarted()) {
            opMode.telemetry.addData("Heading:", driveTrain.getAngle());
            opMode.telemetry.update();
        }
    }
}
