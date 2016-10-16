package org.firstinspires.ftc.teamcode.AutonomousNavigation.Waypoint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.AutonomousNavigation.Waypoint.Transform.Transform;
import org.firstinspires.ftc.teamcode.Util.PID.PidController;

/**
 * Created by hsunx on 10/14/2016.
 */

public class GyroAndVuforiaWaypointNavigator extends WaypointNavigator {

    GyroSensor gyro;

    // Uses PID!
    PidController pidController;

    // Motors
    DcMotor leftMotor;
    DcMotor rightMotor;

    int previousLeftEncoder;
    int previousRightEncoder;

    public GyroAndVuforiaWaypointNavigator (Waypoint initialTarget) {
        super(initialTarget);
    }
    // Drives to target
    @Override
    public void DriveToTarget() {
        OpenGLMatrix matrixPosition = finder.FindPosition();
        if (matrixPosition == null) {
            // Fallback to gyros + encoders

        }

    }
}
