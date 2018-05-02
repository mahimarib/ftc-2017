package org.firstinspires.ftc.teamcode.tools.PID;

/**
 * Created by Mahim on 5/2/18.
 */

public interface PIDSource {
    public enum PIDSourceType {
        Displacement, Rate
    }

    public PIDSourceType getPIDSourceType();

    public double PIDGet();
}
