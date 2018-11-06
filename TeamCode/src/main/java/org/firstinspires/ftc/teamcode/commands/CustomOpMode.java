package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * CustomOpMode is an abstract class that all routines extends. It
 * contains methods and/or instance variables that is common for all routines.
 */
public abstract class CustomOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    protected Robot robot = new Robot(this);

    /**
     * Runs actions that are needed before the match starts. Such as
     * initializing the robot.
     */
    protected abstract void initialize();

    /**
     * Runs actions that are needed during the match.
     */
    protected abstract void execute();

    /**
     * Once this condition is met, the loop stops
     *
     * @return if the actions is finished.
     */
    protected abstract boolean isFinished();

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            execute();
            if (isFinished()) {
                break;
            }
        }
    }
}
