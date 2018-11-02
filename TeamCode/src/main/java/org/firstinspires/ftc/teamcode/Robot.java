package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;
import org.firstinspires.ftc.teamcode.systems.JewelSweeper;
import org.firstinspires.ftc.teamcode.systems.Mechanism;

import static org.firstinspires.ftc.teamcode.systems.Drivetrain.Direction;

public class Robot extends Mechanism {
    public Drivetrain drivetrain;
    public Arm arm;
    public JewelSweeper jewelSweeper;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        arm = new Arm(opMode);
        jewelSweeper = new JewelSweeper(opMode);
    }

    public Robot() {
        drivetrain = new Drivetrain();
        arm = new Arm();
        jewelSweeper = new JewelSweeper();
    }

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        arm.init(hwMap);
        jewelSweeper.init(hwMap);
        jewelSweeper.setInitPosition();
    }

    public void knockDownBlueJewel() {
        if (opMode.opModeIsActive()) {
            jewelSweeper.goDown();
            opMode.sleep(1500);
            if (jewelSweeper.isBlue()) {
                drivetrain.drive(0.5, 0.5, Direction.REVERSE);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else if (jewelSweeper.isRed()) {
                drivetrain.drive(0.5, 0.5, Direction.FORWARD);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else {
                drivetrain.stop();
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
                drivetrain.drive(0.5, 0.5, Direction.FORWARD);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else if (jewelSweeper.isRed()) {
                drivetrain.drive(0.5, 0.5, Direction.REVERSE);
                //TODO: check if it's the right direction
                opMode.sleep(250);
            } else {
                drivetrain.stop();
            }
            jewelSweeper.setInitPosition();
            opMode.sleep(1000);
        }
    }
}
