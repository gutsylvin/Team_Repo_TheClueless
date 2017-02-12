package org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by Josh on 2/11/2017.
 */

public abstract class DriveTrain {

    //region driving
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Temporary solution, based on relative CPR's of Neverest 40 and Tetrix Max Dc Motors for use while we
    // are migrating from Tetrix to andymark
    static final double MOVEMENT_FACTOR = 0.7777777777777;
    static final double SPEED_FACTOR = 0.95;

    static final double LINE_FOLLOW_TARGET = 0.4;


    static final double CONFIDENCE_THRESHOLD = 0.75; // This is how confident the FTCVision reading needs to be in order
    // for the program to be sure of its results. Lower is less accurate (duh)
    // but higher may lead to inefficiency.

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double TURN_SPEED = 0.15;     // Nominal half speed for better accuracy.
    static final double TURN_TIMEOUT = 10;

    static final double START_ENCODER = 200;    // Driving subroutines will use this encoder value as the "revving up" period
    static final double END_ENCODER = 900;

    static final double HEADING_THRESHOLD = 2;      // (NOT) As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.06;     // Larger is more responsive, but also less stable
    static final double P_LINE_COEFF = 0.25;
    //endregion

    static final double TIMEOUT_MULTIPLIER = 1;

    public abstract void vuforiaDrive(double speed, VuforiaTransform position) throws InterruptedException;

    public abstract void gyroTurn(double speed, double target) throws InterruptedException;
    public abstract void encoderTurn(double speed, double encoderAmount) throws InterruptedException;

    public class VuforiaTransform {
        public double x;
        public double y;
        public double z;

        // Measured in Euler angles (yay no quaternions)
        public double x_rotation;
        public double y_rotation;
        public double z_rotation;
    }
}
