package org.firstinspires.ftc.teamcode.customdrive;

import com.roboracers.topgear.controls.PIDCoefficients;

public class TuneableConstants {

    /*
     * The PID coefficients for the x, y, and heading controllers, used to bring the robot to a stop at the end of GVF following.
     */
    /**
     * The PID coefficients for the x controller.
     */
    public static PIDCoefficients xPIDCoeffs = new PIDCoefficients(1,0,0);
    /**
     * The PID coefficients for the y controller.
     */
    public static PIDCoefficients yPIDCoeffs = new PIDCoefficients(1,0,0);
    /**
     * The PID coefficients for the heading controller.
     */
    public static PIDCoefficients headingPIDCoeffs = new PIDCoefficients(0,0,0);
    /**
     * The distance between the closest point and the tangent point, measured in inches.
     */
    public static double tangentDistance = 1;
    /**
     * Max speed of the robot while following the path.
     * Measured between 0 and 1.
     */
    public static double maxSpeed = 1;
    /**
     * Threshold for the end PID to kick in, measured in inches.
     */
    public static double PIDThreshold = 1;

    /**
     * The default curvature for the bezier curve builder..
     */
    public static double DEFAULT_CURVATURE = 0.5;



}
