package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.lasarobotics.library.util.Log;

/**
 * Created by hsunx on 12/16/2016.
 */

@TeleOp (name = "Turn Tester", group = "ayy kek lmao")
public class TurnTester extends LinearOpMode {
    Robot robot = new Robot();

    public static double turnSpeed;

    static final double TURN_SPEED = 0.15;     // Nominal half speed for better accuracy.
    static final double TURN_TIMEOUT = 10;

    static final double START_ENCODER = 200;    // Driving subroutines will use this encoder value as the "revving up" period
    static final double END_ENCODER = 500;

    static final double SPEED_FACTOR = 0.95;

    static final double HEADING_THRESHOLD = 2;      // (NOT) As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.06;

    Log logger;

    boolean shooting;
    ElapsedTime timer;

    Gamepad previousGamepad1;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        previousGamepad1 = new Gamepad();
        timer = new ElapsedTime();
        logger = new Log("FIRST", "Shooter_test_log");
        telemetry.update();
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double leftDiff = 0;
            double rightDiff = 0;
            double totalTime = 0;

            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                turnSpeed += 0.01;
            }
            if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                turnSpeed -= 0.01;
            }

            telemetry.addData("Turn Speed", turnSpeed);
            telemetry.addData("Current Angle", robot.gyro.getHeading());
            telemetry.update();

            if (gamepad1.a && !previousGamepad1.a) {
                gyroTurn(turnSpeed, robot.gyro.getHeading() + 90);
            }

            try {
                previousGamepad1.copy(gamepad1);
            }
            catch (RobotCoreException r) {
                // Who the fuck cares
            }
        }
    }

    public void gyroTurn(double speed, double angle)
            throws InterruptedException {
        speed *= SPEED_FACTOR;
        double startTime = time;
        // keep looping while we are still active, and not on heading.

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            if (time > TURN_TIMEOUT + startTime) {
                break;
            }

            telemetry.addData("time", time);
            telemetry.addData("timeout", TURN_TIMEOUT * 1000);
            telemetry.addData("left encoder", robot.leftMotor.getCurrentPosition());
            telemetry.addData("right encoder", robot.rightMotor.getCurrentPosition());
            RobotLog.i("left encoder : " + robot.leftMotor.getCurrentPosition());
            RobotLog.i("right encoder : " + robot.rightMotor.getCurrentPosition());
            telemetry.addData("gyro", robot.gyro.getHeading());
            telemetry.update();
            idle();

        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    // returns whether a is within a range of c relative to b
    public boolean within (double a, double b, double c) {
        return a >= b+c && a <= b-c;
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer * SPEED_FACTOR;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
