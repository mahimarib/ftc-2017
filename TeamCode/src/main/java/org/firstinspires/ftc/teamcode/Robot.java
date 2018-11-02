package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.Mechanism;

import static org.firstinspires.ftc.teamcode.systems.DriveSystem.Direction;

public class Robot extends Mechanism {
    public DriveSystem driveSystem;
    public ArmSystem armSystem;
    public ColorSensorSystem colorSensorSystem;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        driveSystem = new DriveSystem(opMode);
        armSystem = new ArmSystem(opMode);
        colorSensorSystem = new ColorSensorSystem(opMode);
    }

    public Robot() {
        driveSystem = new DriveSystem();
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
        if (opMode.opModeIsActive()) {
            colorSensorSystem.goDown();
            opMode.sleep(1500);
            if (colorSensorSystem.isBlue()) {
                driveSystem.drive(0.5, 0.5, Direction.REVERSE);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else if (colorSensorSystem.isRed()) {
                driveSystem.drive(0.5, 0.5, Direction.FORWARD);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else {
                driveSystem.stop();
            }
            colorSensorSystem.setInitPosition();
            opMode.sleep(1000);
        }
    }

    public void knockDownRedJewel() {
        if (opMode.opModeIsActive()) {
            colorSensorSystem.goDown();
            opMode.sleep(1500);
            if (colorSensorSystem.isBlue()) {
                driveSystem.drive(0.5, 0.5, Direction.FORWARD);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else if (colorSensorSystem.isRed()) {
                driveSystem.drive(0.5, 0.5, Direction.REVERSE);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else {
                driveSystem.stop();
            }
            colorSensorSystem.setInitPosition();
            opMode.sleep(1000);
        }
    }
}
