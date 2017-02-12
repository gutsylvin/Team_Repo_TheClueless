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

        vpBeacon(0.25);
    }

    // Left to right
    enum BeaconState {
        REDBLUE,
        BLUERED,
        BLUEBLUE,
        REDRED,
        UNKNOWN
    }

    //region beacon
    public void pushBasedOnState(BeaconState state) {
        if (state == BeaconState.REDBLUE) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(true);
            }
            else {
                robot.push(false);
            }
        }
        else if (state == BeaconState.BLUERED) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(false);
            }
            else {
                robot.push(true);
            }
        }
        else if (state == BeaconState.BLUEBLUE) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.pushBoth(true);
            }
        }
        else if (state == BeaconState.REDRED) {
            if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
                robot.pushBoth(true);
            }
        }
    }

    // Assumes both sides are different colors
    BeaconState getBeaconState() {
        Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
        RobotLog.d(analysis.toString());

        BeaconState visionState = BeaconState.UNKNOWN;
        BeaconState sensorState = BeaconState.UNKNOWN;

        if (analysis.isLeftKnown() && analysis.isRightKnown()) {
            if (analysis.isLeftBlue() && analysis.isRightRed()) {
                visionState = BeaconState.BLUERED;
            }
            else if (analysis.isLeftRed() && analysis.isRightBlue()) {
                visionState = BeaconState.REDBLUE;
            }
            else if (analysis.isLeftBlue() && analysis.isRightBlue()) {
                visionState = BeaconState.BLUEBLUE;
            }
            else if (analysis.isRightBlue() && analysis.isRightBlue()) {
                visionState = BeaconState.BLUEBLUE;
            }
            else {
                visionState = BeaconState.UNKNOWN;
            }
        }

        if (!notDetecting(robot.colorSensor)) {
            if (isRed(robot.colorSensor)) {
                sensorState = BeaconState.BLUERED;
            }
            else {
                sensorState = BeaconState.REDBLUE;
            }
        }

        if (visionState == BeaconState.BLUEBLUE || visionState == BeaconState.REDRED) {
            return visionState;
        }

        if (visionState == sensorState) {
            // Good
            return visionState;
        }
        else if (visionState == BeaconState.UNKNOWN) {
            return sensorState;
        }
        else if (sensorState == BeaconState.UNKNOWN) {
            return visionState;
        }
        else {
            return BeaconState.UNKNOWN;
        }
    }

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

        /*boolean retry = true;
        BeaconState afterState = getBeaconState();
        if (afterState == BeaconState.REDRED) {
            if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
                pushBasedOnState(afterState);
            }
            retry = false;
        }
        else if (afterState == BeaconState.BLUEBLUE) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                pushBasedOnState(afterState);
            }
            retry = false;
        }
        else if (afterState != BeaconState.UNKNOWN) {
            //pushBasedOnState(afterState);
            //retry = true;

            retry = false;
        }
        else {
            retry = false;
        }*/
        if ((isRed(robot.colorSensor) && MatchDetails.color == MatchDetails.TeamColor.BLUE)
                || (isBlue(robot.colorSensor) && MatchDetails.color == MatchDetails.TeamColor.RED)) {
            encoderDrive(0.75, -150, -150, 2000);
            robot.pushBoth(true);
            Thread.sleep(5000);
            encoderDrive(0.5, 300, 300, 1750);
            Thread.sleep(300);
            encoderDrive(0.5, -75, -75, 800);
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
                            /*double angle*/, double target) throws InterruptedException {

        gyroDriveUntilLine(speed, speed, target);
    }

    public void gyroDriveUntilLine(double leftSpeed, double rightSpeed
                            /*double angle*/, double target) throws InterruptedException {

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
                if (scaledValue >= target) {
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

    public void stopRobotMotion() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    //endregion
}
