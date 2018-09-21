package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot extends Mechanism {
    public MecanumDriveSystem driveSystem;
    public ArmSystem armSystem;
    public ColorSensorSystem colorSensorSystem;

    public Robot(LinearOpMode opMode) {
        driveSystem = new MecanumDriveSystem(opMode);
        armSystem = new ArmSystem(opMode);
        colorSensorSystem = new ColorSensorSystem(opMode);
    }

    public Robot() {
        driveSystem = new MecanumDriveSystem();
        armSystem = new ArmSystem();
        colorSensorSystem = new ColorSensorSystem();
    }

    @Override
    public void init(HardwareMap hwMap) {
        driveSystem.init(hwMap);
        armSystem.init(hwMap);
        colorSensorSystem.init(hwMap);
    }
}
