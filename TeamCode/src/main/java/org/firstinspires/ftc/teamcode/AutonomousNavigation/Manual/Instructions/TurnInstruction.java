package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions;

import com.google.gson.annotations.Expose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions.Instruction;

/**
 * Created by Josh on 10/30/2016.
 */

public class TurnInstruction extends Instruction {

    @Expose
    public double speed;
    @Expose
    public double angle;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    @Override
    public void Loop() {
        if (onHeading(speed, angle, P_TURN_COEFF))
            Finished();
    }

    @Override
    public void Init() {
        super.Init();
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn speed based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        robot.telemetry.addData("Target", "%5.2f", angle);
        robot.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        robot.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    // TODO Maybe refactor into some sort of GyroUtils class to share these methods, copypasta is rather inefficient
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    @Override
    public String toString() {
        return name + "angle: " + angle + "speed: " + speed;
    }
}
