package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.JewelSweeper;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.Mechanism;

import static org.firstinspires.ftc.teamcode.systems.DriveSystem.Direction;

public class Robot extends Mechanism {
    public DriveSystem driveSystem;
    public ArmSystem armSystem;
    public JewelSweeper jewelSweeper;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        driveSystem = new DriveSystem(opMode);
        armSystem = new ArmSystem(opMode);
        jewelSweeper = new JewelSweeper(opMode);
    }

    public Robot() {
        driveSystem = new DriveSystem();
        armSystem = new ArmSystem();
        jewelSweeper = new JewelSweeper();
    }

    @Override
    public void init(HardwareMap hwMap) {
        driveSystem.init(hwMap);
        armSystem.init(hwMap);
        jewelSweeper.init(hwMap);
        jewelSweeper.setInitPosition();
    }

    public void knockDownBlueJewel() {
        if (opMode.opModeIsActive()) {
            jewelSweeper.goDown();
            opMode.sleep(1500);
            if (jewelSweeper.isBlue()) {
                driveSystem.drive(0.5, 0.5, Direction.REVERSE);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else if (jewelSweeper.isRed()) {
                driveSystem.drive(0.5, 0.5, Direction.FORWARD);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else {
                driveSystem.stop();
            }
            jewelSweeper.setInitPosition();
            opMode.sleep(1000);
        }
    }

    public void knockDownRedJewel() {
        if (opMode.opModeIsActive()) {
            jewelSweeper.goDown();
            opMode.sleep(1500);
            if (jewelSweeper.isBlue()) {
                driveSystem.drive(0.5, 0.5, Direction.FORWARD);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else if (jewelSweeper.isRed()) {
                driveSystem.drive(0.5, 0.5, Direction.REVERSE);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else {
                driveSystem.stop();
            }
            jewelSweeper.setInitPosition();
            opMode.sleep(1000);
        }
    }
}
