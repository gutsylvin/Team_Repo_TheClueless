package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.MatchDetails;
import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import static org.firstinspires.ftc.teamcode.AutonomousOpModes.AutonomousOpMode.HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.AutonomousOpModes.AutonomousOpMode.P_TURN_COEFF;
import static org.firstinspires.ftc.teamcode.RobotHardware.Robot.robot;

/**
 * Created by hsunx on 2/10/2017.
 */
@Autonomous(name = "Beacon test", group = "test")
public class BeaconTest extends LinearVisionOpMode {
    //region bloated shit
    enum BeaconTestType {
        VISIONPROCESSING,
        COLORSENSOR,
        RESET
    }
    private Beacon localBeacon;
    private BeaconTestType type = BeaconTestType.COLORSENSOR;
    private ElapsedTime runtime = new ElapsedTime();
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
    //endregion
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap, telemetry);
        localBeacon = new Beacon(Beacon.AnalysisMethod.COMPLEX);

        //region ftcvision
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        this.setCamera(Cameras.PRIMARY);

        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        //endregion

        waitForStart();

        //gyroDriveUntilLine(0.075, 0.4, 6.2);
        //gyroTurn(0.2, 90);
        vpBeacon(0.125);
    }

    // Left to right
    enum BeaconState {
        REDBLUE,
        BLUERED,
        BLUEBLUE,
        REDRED,
        UNKNOWN
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
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

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
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
    //region beacon

    void vpBeacon(double speed) throws InterruptedException {
        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Team Color", MatchDetails.color);

        Thread.sleep(200);

        BeaconState state = getBeaconState();

        while (state == BeaconState.UNKNOWN) {
            encoderDrive(speed, 150, 2250);
            Thread.sleep(200);
            state = getBeaconState();
        }

        pushBasedOnState(state);

        encoderDrive(speed, 800, 3000);

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.pushBoth(false);

        Thread.sleep(200);

        boolean retry = true;
        BeaconState afterState = getBeaconState();
        if (afterState == BeaconState.REDRED) {
            if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
                pushBasedOnState(afterState);
            }
            else {
                retry = false;
            }
        }
        else if (afterState == BeaconState.BLUEBLUE) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                pushBasedOnState(afterState);
            }
            else {
                retry = false;
            }
        }
        else if ((isRed(robot.colorSensor) && MatchDetails.color == MatchDetails.TeamColor.BLUE) || (isBlue(robot.colorSensor) && MatchDetails.color == MatchDetails.TeamColor.RED)) {
            retry = true;
        }
        else {
            retry = false;
        }

        if (retry) {
            encoderDrive(0.75, -150, -150, 2000);
            robot.pushBoth(true);
            Thread.sleep(5000);
            encoderDrive(0.5, 300, 300, 1750);
            Thread.sleep(300);
            encoderDrive(0.5, -75, -75, 800);
        }
    }

    public void pushBasedOnState(BeaconTest.BeaconState state) {
        if (state == BeaconTest.BeaconState.REDBLUE) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(true);
            }
            else {
                robot.push(false);
            }
        }
        else if (state == BeaconTest.BeaconState.BLUERED) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(false);
            }
            else {
                robot.push(true);
            }
        }
        else if (state == BeaconTest.BeaconState.BLUEBLUE) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.pushBoth(true);
            }
        }
        else if (state == BeaconTest.BeaconState.REDRED) {
            if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
                robot.pushBoth(true);
            }
        }
    }

    // Assumes both sides are different colors
    BeaconTest.BeaconState getBeaconState() {
        Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
        RobotLog.d(analysis.toString());

        BeaconTest.BeaconState visionState = BeaconTest.BeaconState.UNKNOWN;
        BeaconTest.BeaconState sensorState = BeaconTest.BeaconState.UNKNOWN;

        if (analysis.isLeftKnown() && analysis.isRightKnown()) {
            if (analysis.isLeftBlue() && analysis.isRightRed()) {
                visionState = BeaconTest.BeaconState.BLUERED;
            }
            else if (analysis.isLeftRed() && analysis.isRightBlue()) {
                visionState = BeaconTest.BeaconState.REDBLUE;
            }
            else if (analysis.isLeftRed() && analysis.isRightRed()) {
                visionState = BeaconTest.BeaconState.REDRED;
            }
            else if (analysis.isRightBlue() && analysis.isRightBlue()) {
                visionState = BeaconTest.BeaconState.BLUEBLUE;
            }
            else {
                visionState = BeaconTest.BeaconState.UNKNOWN;
            }
        }

        if (!notDetecting(robot.colorSensor)) {
            if (isRed(robot.colorSensor)) {
                sensorState = BeaconTest.BeaconState.BLUERED;
            }
            else {
                sensorState = BeaconTest.BeaconState.REDBLUE;
            }
        }

        if (visionState == BeaconTest.BeaconState.BLUEBLUE || visionState == BeaconTest.BeaconState.REDRED) {
            return visionState;
        }

        if (visionState == sensorState) {
            // Good
            return visionState;
        }
        else if (visionState == BeaconTest.BeaconState.UNKNOWN) {
            return sensorState;
        }
        else if (sensorState == BeaconTest.BeaconState.UNKNOWN) {
            return visionState;
        }
        else {
            return BeaconTest.BeaconState.UNKNOWN;
        }
    }




    boolean isRed (ColorSensor c) {
        if (c.red() > c.blue()) {
            return true;
        }
        return false;
    }

    boolean isBlue (ColorSensor c) {
        if (c.blue() > c.red()) {
            return true;
        }
        return false;
    }
    public boolean reviseColor () {

            if (isRed(robot.colorSensor)) {
                if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                    robot.push(false);
                } else {
                    robot.push(true);
                }
                return true;
            } else if (isBlue(robot.colorSensor)) {
                if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                    robot.push(true);
                } else {
                    robot.push(false);
                }
                return true;
            }
            //}
            return false;

        //return (isRed(robot.colorSensor) && isRed(robot.adafruitColorSensor) || (isBlue(robot.colorSensor) && isBlue(robot.adafruitColorSensor)));
    }

    public boolean notDetecting(ColorSensor c) {
        return c.blue() == c.red();
    }
    //endregion

    //region driving
    public void encoderDrive(double speed,
                             double left, double right,
                             double timeoutMs) throws InterruptedException {
        encoderDrive(speed, speed, speed, left, right, timeoutMs, true);
    }

    public void encoderDrive(double speed, double counts, double timeoutMs) throws InterruptedException {
        encoderDrive(speed, speed, speed, counts, counts, timeoutMs, true);
    }

    public void encoderDriveColor(double speed,
                                  double counts,
                                  double timeoutMs) throws InterruptedException {
        encoderDriveColor(speed, speed, speed, counts, counts, timeoutMs);
    }

    public void encoderDriveColor(double speed,
                                  double speed1,
                                  double speed2,
                                  double left, double right,
                                  double timeoutMs) throws InterruptedException{
        int newLeftTarget;
        int newRightTarget;
        int encoderStartLeft;
        int encoderStartRight;
        boolean stopped = false;

        speed *= SPEED_FACTOR;
        speed1 *= SPEED_FACTOR;
        speed2 *= SPEED_FACTOR;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

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

            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeoutMs) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                boolean pastStart = (Math.abs(robot.leftMotor.getCurrentPosition() - encoderStartLeft) > START_ENCODER
                        || Math.abs(robot.rightMotor.getCurrentPosition() - encoderStartRight) > START_ENCODER);

                boolean pastEnd = (Math.abs(newLeftTarget - robot.leftMotor.getCurrentPosition()) < END_ENCODER
                        || Math.abs(newRightTarget - robot.rightMotor.getCurrentPosition()) < END_ENCODER);

                boolean stop = reviseColor();
                if (stop && !stopped) {
                    stopRobotMotion();
                    stopped = true;
                    Thread.sleep(200);
                    continue;
                }

                robot.leftMotor.setPower(Math.abs(pastEnd ? speed2 : (pastStart ? speed1 : speed)));
                robot.rightMotor.setPower(Math.abs(pastEnd ? speed2 : (pastStart ? speed1 : speed)));
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Gyro Angle", robot.gyro.getHeading());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
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

    public void encoderDrive(double speed,
                             double speed1,
                             double speed2,
                             double left, double right,
                             double timeoutMs, boolean endBreak) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int encoderStartLeft;
        int encoderStartRight;

        speed *= SPEED_FACTOR;
        speed1 *= SPEED_FACTOR;
        speed2 *= SPEED_FACTOR;

        robot.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

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

            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutMs) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                boolean pastStart = (Math.abs(robot.leftMotor.getCurrentPosition() - encoderStartLeft) > START_ENCODER
                        || Math.abs(robot.rightMotor.getCurrentPosition() - encoderStartRight) > START_ENCODER);

                boolean pastEnd = (Math.abs(newLeftTarget - robot.leftMotor.getCurrentPosition()) < END_ENCODER
                        || Math.abs(newRightTarget - robot.rightMotor.getCurrentPosition()) < END_ENCODER);

                robot.leftMotor.setPower(Math.abs(pastEnd ? speed2 : (pastStart ? speed1 : speed)));
                robot.rightMotor.setPower(Math.abs(pastEnd ? speed2 : (pastStart ? speed1 : speed)));
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Gyro Angle", robot.gyro.getHeading());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
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
    /**
     * @param speed     speed
     * @param target    target for the optical distance sensor
     * @throws InterruptedException
     */

    public void gyroDriveUntilLine(double speed
                            /*double angle*/, double target, double adafruitTarget) throws InterruptedException {

        gyroDriveUntilLine(speed, speed, target, adafruitTarget);
    }

    public void gyroDriveUntilLine(double leftSpeed, double rightSpeed
                            /*double angle*/, double target, double adafruitTarget) throws InterruptedException {

        runtime.reset();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // start motion.
            leftSpeed = Range.clip(leftSpeed, -1.0, 1.0) * SPEED_FACTOR;
            rightSpeed = Range.clip(rightSpeed, -1.0, 1.0) * SPEED_FACTOR;
            robot.leftMotor.setPower(leftSpeed);
            robot.rightMotor.setPower(rightSpeed);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive()) {
                double scaledValue = robot.opticalDistanceSensor.getLightDetected();
                telemetry.addData("light", scaledValue);
                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.update();
                if (scaledValue >= target || (getGreyscale(robot.adafruitI2cColorSensor) / 255 > adafruitTarget)) {
                    stopRobotMotion();
                    return;
                }
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public double getGreyscale(ColorSensor c) {
        return 0.299 * c.red() + 0.587 * c.green() + 0.114 * c.blue();
    }

    public void stopRobotMotion() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    //endregion
}
