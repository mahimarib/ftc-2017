package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.Mechanism;

import static org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem.Direction.REVERSE;

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
        colorSensorSystem.setInitPosition();
    }

    public void knockDownBlueJewel() {
        colorSensorSystem.goDown();
        linearOpMode.sleep(1500);
        if(colorSensorSystem.isBlue()) {
            driveSystem.drive(0.5, 0.5, REVERSE);
            linearOpMode.sleep(250);
        } else if(colorSensorSystem.isRed()) {
            driveSystem.drive(0.5, 0.5, FORWARD);
            linearOpMode.sleep(250);
        } else {
            driveSystem.stop();
        }
        colorSensorSystem.setInitPosition();
        linearOpMode.sleep(1000);
    }

    public void knockDownRedJewel() {
        colorSensorSystem.goDown();
        linearOpMode.sleep(1500);
        if(colorSensorSystem.isBlue()) {
            driveSystem.drive(0.5, 0.5, REVERSE);
            linearOpMode.sleep(250);
        } else if(colorSensorSystem.isRed()) {
            driveSystem.drive(0.5, 0.5, FORWARD);
            linearOpMode.sleep(250);
        } else {
            driveSystem.stop();
        }
        colorSensorSystem.setInitPosition();
        linearOpMode.sleep(1000);
    }
}
