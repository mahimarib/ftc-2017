package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;
import org.firstinspires.ftc.teamcode.systems.JewelSweeper;
import org.firstinspires.ftc.teamcode.systems.Mechanism;

import static org.firstinspires.ftc.teamcode.systems.Drivetrain.Direction;

/**
 * The Robot class holds all the physical component of the robot. This class
 * contains autonomous actions involving multiple robot mechanisms. These
 * actions may be common to more than one routine.
 */
public class Robot extends Mechanism {
    // instance variable of the robot's drivetrain
    public Drivetrain drivetrain;

    // instance variable of the robot's arm
    public Arm arm;

    // instance variable of the robot's jewel sweeper
    public JewelSweeper jewelSweeper;

    /**
     * Sets the the opMode to the current OpMode that is running, this is to
     * make use of certain functionalities that the OpMode provides, such as
     * the {@link LinearOpMode#sleep(long)}.
     *
     * @param opMode the current LinearOpMode that is running.
     */
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        arm = new Arm(opMode);
        jewelSweeper = new JewelSweeper(opMode);
    }

    /**
     * Default constructor for Robot. Instantiates public instances for the
     * robot's mechanisms.
     */
    public Robot() {
        drivetrain = new Drivetrain();
        arm = new Arm();
        jewelSweeper = new JewelSweeper();
    }

    /**
     * Initializes all the robot's mechanisms, as well as set them in position.
     *
     * @param hwMap robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        arm.init(hwMap);
        jewelSweeper.init(hwMap);
        jewelSweeper.setInitPosition();
    }

    /**
     * Knocks down the blue jewel, uses more than one mechanism, this is used
     * commonly in more than 1 routine.
     */
    public void knockDownBlueJewel() {
        if (opMode.opModeIsActive()) {
            // set the jewel arm in position to check the color of the jewel.
            jewelSweeper.goDown();
            // waits 1.5 seconds before checking
            // this is to make sure it does not read color values that are
            // not the jewel.
            opMode.sleep(1500);
            if (jewelSweeper.isBlue()) {
                // if the jewel is blue it will drive backwards since the
                // jewel sweeper is actually reverse and check the jewel that
                // is in the back.
                drivetrain.drive(0.5, 0.5, Direction.REVERSE);
                // TODO: check if it's the right direction
                // drives for quarter of a second.
                opMode.sleep(250);
            } else if (jewelSweeper.isRed()) {
                // if the jewel in the back is red, it will drive forward to
                // knock down the blue jewel that is in the front.
                drivetrain.drive(0.5, 0.5, Direction.FORWARD);
                // TODO: check if it's the right direction
                // drives for quarter of a second.
                opMode.sleep(250);
            } else {
                // if it detects nothing it won't do anything.
                drivetrain.stop();
            }
            // the arm goes back in initial position for the rest of the match.
            jewelSweeper.setInitPosition();
            opMode.sleep(1000);
        }
    }

    /**
     * Knocks down the red jewel, uses more than one mechanism, this is used
     * commonly in more than 1 routine.
     */
    public void knockDownRedJewel() {
        if (opMode.opModeIsActive()) {
            // set the jewel arm in position to check the color of the jewel.
            jewelSweeper.goDown();
            // waits 1.5 seconds before checking
            // this is to make sure it does not read color values that are
            // not the jewel.
            opMode.sleep(1500);
            if (jewelSweeper.isBlue()) {
                // if the jewel is blue it will drive forwards since the
                // jewel sweeper is actually reversed and checks the jewel in
                // the back, it will drive forwards since the jewel in the
                // front is the red jewel.
                drivetrain.drive(0.5, 0.5, Direction.FORWARD);
                // TODO: check if it's the right direction
                // drives for quarter of a second.
                opMode.sleep(250);
            } else if (jewelSweeper.isRed()) {
                // if the jewel in the back is red, it will drive reverse to
                // knock it down.
                drivetrain.drive(0.5, 0.5, Direction.REVERSE);
                // TODO: check if it's the right direction
                // drives for quarter of a second.
                opMode.sleep(250);
            } else {
                // it if detects nothing it won't do anything.
                drivetrain.stop();
            }
            // the arm goes back in initial position for the rest of the match.
            jewelSweeper.setInitPosition();
            opMode.sleep(1000);
        }
    }
}
