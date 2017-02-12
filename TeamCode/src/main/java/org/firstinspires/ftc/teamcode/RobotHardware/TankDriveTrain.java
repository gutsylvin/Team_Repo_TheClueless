package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.Callable;

import static org.firstinspires.ftc.teamcode.RobotHardware.GyroUtils.onHeading;
import static org.firstinspires.ftc.teamcode.RobotHardware.Robot.robot;

/**
 * Created by Josh on 2/11/2017.
 */

public class TankDriveTrain extends DriveTrain {
    ElapsedTime runtime;

    public TankDriveTrain() {
        runtime = new ElapsedTime();
    }

    public void encoderDrive(double speed,
                             double left, double right,
                             double timeoutMs) throws InterruptedException {
        encoderDrive(speed, speed, speed, left, right, timeoutMs, true, null);
    }

    public void encoderDrive(double speed, double counts, double timeoutMs) throws InterruptedException {
        encoderDrive(speed, speed, speed, counts, counts, timeoutMs, true, null);
    }

    public void encoderDriveUntilLine(double speed, double timeoutMs, final double target) throws InterruptedException{
        encoderDrive(speed, speed, speed, 100000, 100000, timeoutMs, true, new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return robot.opticalDistanceSensor.getLightDetected() > target;
            }
        });
    }

    public void encoderDrive(double speed,
                             double speed1,
                             double speed2,
                             double left, double right,
                             double timeoutMs, boolean endBrake, Callable<Boolean> stopCondition) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int encoderStartLeft;
        int encoderStartRight;

        robot.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that the opmode is still active
        if (robot.notStopped()) {

            encoderStartLeft = robot.leftMotor.getCurrentPosition();
            encoderStartRight = robot.rightMotor.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            newLeftTarget = encoderStartLeft + (int) ((left));
            newRightTarget = encoderStartRight + (int) ((right));

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (endBrake) {
                robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            boolean ignoreStopCondition = stopCondition == null;

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (robot.notStopped() &&
                    (runtime.seconds() < timeoutMs) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                if (!ignoreStopCondition) {
                    try {
                        if (stopCondition.call()) {
                            break;
                        }
                    }
                    catch (Exception e) {

                    }
                }
                boolean pastStart = (Math.abs(robot.leftMotor.getCurrentPosition() - encoderStartLeft) > START_ENCODER
                        || Math.abs(robot.rightMotor.getCurrentPosition() - encoderStartRight) > START_ENCODER);

                boolean pastEnd = (Math.abs(newLeftTarget - robot.leftMotor.getCurrentPosition()) < END_ENCODER
                        || Math.abs(newRightTarget - robot.rightMotor.getCurrentPosition()) < END_ENCODER);

                robot.leftMotor.setPower(Math.abs(pastEnd ? speed2 : (pastStart ? speed1 : speed)));
                robot.rightMotor.setPower(Math.abs(pastEnd ? speed2 : (pastStart ? speed1 : speed)));
                // Display it for the driver.
                robot.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                robot.telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                robot.telemetry.addData("Gyro Angle", robot.gyro.getHeading());
                robot.telemetry.update();

                // Allow time for other processes to run.
                robot.currentOpMode.idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    @Override
    public void vuforiaDrive(double speed, VuforiaTransform position) throws InterruptedException {
        // TODO
    }

    @Override
    public void gyroTurn(double speed, double angle)
            throws InterruptedException {

        speed *= SPEED_FACTOR;
        double startTime = robot.currentOpMode.time;
        // keep looping while we are still active, and not on heading.
        while (robot.notStopped() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            if (robot.currentOpMode.time > TURN_TIMEOUT + startTime) {
                break;
            }
            robot.telemetry.addData("time", robot.currentOpMode.time);
            robot.telemetry.addData("timeout", TURN_TIMEOUT * 1000);
            robot.telemetry.addData("left encoder", robot.leftMotor.getCurrentPosition());
            robot.telemetry.addData("right encoder", robot.rightMotor.getCurrentPosition());
            RobotLog.i("left encoder : " + robot.leftMotor.getCurrentPosition());
            RobotLog.i("right encoder : " + robot.rightMotor.getCurrentPosition());
            robot.telemetry.addData("gyro", robot.gyro.getHeading());
            robot.telemetry.update();
            robot.currentOpMode.idle();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    @Override
    public void encoderTurn(double speed, double encoderAmounts) throws InterruptedException {
        encoderDrive(speed, encoderAmounts, -encoderAmounts, 6000);
    }
}
