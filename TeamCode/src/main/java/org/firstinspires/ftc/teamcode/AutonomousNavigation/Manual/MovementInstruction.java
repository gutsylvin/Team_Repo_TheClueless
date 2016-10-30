package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import java.util.Map;

/**
 * Created by hsunx on 10/21/2016.
 */

public class MovementInstruction extends Instruction {
    public final float P_DRIVE_COEFF = 0.15f;

    public float angle;
    public float distance;
    public float power;

    int     newLeftTarget;
    int     newRightTarget;
    int     moveCounts;
    double  max;
    double  error;
    double  steer;
    double  leftSpeed;
    double  rightSpeed;

    @Override
    public void FromMap(Map<String, String> map) {
        if ("Novement" == map.get("type")) {
            // Very good
            try {
                angle = Float.parseFloat(map.get("angle"));
                distance = Float.parseFloat(map.get("distance"));
                power = Float.parseFloat(map.get("power"));
            }
            catch (NumberFormatException n) {
                RobotLog.e("Wtf that isn't even a float");
                n.printStackTrace();
            }
        }
        else {
            // Well crap
            RobotLog.e("SHIIIIT THERE IS NO TYPE YOU DINGUSMASTER50000");
        }
    }

    @Override
    public void Init() {
        super.Init();

        // Determine new target position, and pass to motor controller

        // TODO Is using COUNTS_PER_INCH reliable?
        moveCounts = (int)(distance /** COUNTS_PER_INCH*/);
        newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
        newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        power = Range.clip(Math.abs(power), 0.0f, 1.0f);
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
    }

    @Override
    public void Loop() {
        if (robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = power - steer;
            rightSpeed = power + steer;

            // Normalize speeds if any one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.leftMotor.setPower(leftSpeed);
            robot.rightMotor.setPower(rightSpeed);

            // Display drive status for the driver.
            robot.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            robot.telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            robot.telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            robot.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            robot.telemetry.update();
        }
        else {
            Finished();
        }
    }

    @Override
    public void Finished() {
        finished = true;

        // Stop all motion
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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

}
