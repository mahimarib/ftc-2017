package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.hardware.systems.template.Mechanism;

/**
 * Created by Mahim on 6/16/18.
 */

public class Robot extends Mechanism {
    public DriveSystem driveSystem;

    public Robot() {
        driveSystem = new DriveSystem();
    }

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        driveSystem = new DriveSystem(opMode);
    }

    @Override
    public void init(HardwareMap hwMap) {
        driveSystem.init(hwMap);
    }

}
