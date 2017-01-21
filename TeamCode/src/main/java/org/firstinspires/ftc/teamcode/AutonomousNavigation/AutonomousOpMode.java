package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.firstinspires.ftc.teamcode.ShooterCalibration;
import org.firstinspires.ftc.robotcontroller.internal.EmotionEngine.EmotionManager;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;


/**
 * Created by hsunx on 10/22/2016.
 */

@Autonomous(name = "Autonomous: Autonomous", group = "Autonomous")
public class AutonomousOpMode extends LinearVisionOpMode {
    /* Declare OpMode members. */
    Robot robot;
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    private ElapsedTime runtime = new ElapsedTime();

    //region shooting
    final double timeoutMs = 15.5 * 1000;
    final long shootTime = 2 * 1000;
    //endregion

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
    static final double END_ENCODER = 500;

    static final double HEADING_THRESHOLD = 2;      // (NOT) As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.06;     // Larger is more responsive, but also less stable
    static final double P_LINE_COEFF = 0.25;
    //endregion

    //region main
    @Override
    public void runOpMode() throws InterruptedException {
        //region ftcvision
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        // EmotionManager.initializeTTS();
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


        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */

        robot = new Robot();
        robot.init(hardwareMap, telemetry);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961); // 245/255

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.conveyorGate.setPosition(0.625);

        if (ShooterCalibration.SHOOTING_SPEED == 0) {
            EmotionManager.speak("YOU SCREWED UP, NO CALIBRATION PERFORMED!");
        }

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..

        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData("Team Color", MatchDetails.color);
            telemetry.addData("right servo position", robot.rightPushServo.getPosition());
            telemetry.update();
            if (ShooterCalibration.SHOOTING_SPEED == 0) {
                EmotionManager.speak("YOU SCREWED UP, NO CALIBRATION PERFORMED!");
                return;
            }
            idle();
        }

        if (MatchDetails.color == MatchDetails.TeamColor.RED) {
            encoderDrive(0.30, 0.5, 0.20, 1900, 1900, 8000, false);

            gyroDriveUntilLine(0.1, 0.1, 0.45);

            gyroTurn(TURN_SPEED * 0.65, -93);
            robot.shoot(true, ShooterCalibration.SHOOTING_SPEED);

            pushBeacon(0.125, 90);
            Thread.sleep(500);

            robot.conveyorMotor.setPower(1);
            Thread.sleep(shootTime);
            robot.conveyorMotor.setPower(0);

            robot.leftShootMotor.setPower(0);
            robot.rightShootMotor.setPower(0);

            encoderDrive(0.7, -250, -250, 3000);
            gyroTurn(TURN_SPEED * 0.65, 180);
            encoderDrive(0.5, 0.75, 0.25, 1500, 1500, 10000, false);
            gyroDriveUntilLine(0.125, 0.1, 0.45);

            encoderDrive(0.5, 100, 2000);

            gyroTurn(TURN_SPEED * 0.65, -90);

            pushBeacon(0.125, 90);

            encoderDrive(0.8, -250, -250, 3000);
            gyroTurn(TURN_SPEED, -128);
            encoderDrive(1, -1640, -1640, 6000);

        } else if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {

            encoderDrive(0.25, 0.5, 0.20, 1900, 1900, 8000, false);

            gyroDriveUntilLine(0.1, 0.1, 0.45);

            gyroTurn(TURN_SPEED * 0.65, 93);
            encoderDrive(0.5, 75, 1500);

            robot.shoot(true, ShooterCalibration.SHOOTING_SPEED);
            pushBeacon(0.125, 90);
            Thread.sleep(500);

            robot.conveyorMotor.setPower(1);
            Thread.sleep(shootTime);
            robot.conveyorMotor.setPower(0);

            robot.shoot(false);

            encoderDrive(0.7, -250, -250, 3000);
            gyroTurn(TURN_SPEED * 0.65, 168);
            encoderDrive(0.5, 0.75, 0.25, 1700, 1500, 10000, false);
            gyroDriveUntilLine(0.145, 0.1, 0.45);

            encoderDrive(0.5, 100, 2000);
            gyroTurn(TURN_SPEED * 0.65, 90);
            pushBeacon(0.125, 90);

            encoderDrive(0.8, -250, -250, 3000);
            gyroTurn(TURN_SPEED, 125);
            encoderDrive(1, -1700, -1700, 6000);
        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    //endregion

    //region drivingfunctions
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
            robot.leftMotor.setPower(Math.abs(speed * SPEED_FACTOR));
            robot.rightMotor.setPower(Math.abs(speed * SPEED_FACTOR));

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
                    idle();
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
            robot.leftMotor.setPower(speed * SPEED_FACTOR);
            robot.rightMotor.setPower(speed * SPEED_FACTOR);

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
    //endregion

    //region beacon
    void pushBeacon(double speed, double angle) throws InterruptedException {

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Team Color", MatchDetails.color);

        encoderDriveColor(speed, 1000, 2250);

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961);
    }



    void driveCenteredBeacon(double speed, int timeoutMs, double target) {
        boolean reachedBeacon = false;
        double startTime = time * 1000;

        boolean leftRed = false;

        // Should we use the left side or the right side? -- Note, this is only called once,
        // so if the initial analysis is wrong, uhhh, that's awkward. We just gave our opponents 30 points
        // TODO fix that.
        Beacon.BeaconAnalysis preAnalysis = beacon.getAnalysis();
        if (preAnalysis.getConfidence() > CONFIDENCE_THRESHOLD) {
            if (preAnalysis.isLeftKnown()) {
                if (preAnalysis.isLeftBlue()) {
                    leftRed = false;
                } else {
                    // It's red.
                    leftRed = true;
                }
            } else if (preAnalysis.isRightKnown()) {
                if (preAnalysis.isRightBlue()) {
                    leftRed = true;
                } else {
                    leftRed = false;
                }
            }
        } else {
            // Try again.
            // TODO Make it so the robot flails around trying to find a high confidence result.
        }

        if (leftRed) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(true);
            } else {
                robot.push(false);
            }
        } else {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(false);
            } else {
                robot.push(true);
            }
        }

        while (opModeIsActive() && !reachedBeacon && time * 1000 - startTime > timeoutMs) {

            // Check if we need to break out of the loop
            Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
            if (analysis.isBeaconFullyBlue()) {
                if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {
                    // Very good
                    break;
                } else {
                    // fuck
                    // TODO Try again, this is a disaster
                }
            }
            if (analysis.isBeaconFullyRed()) {
                if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                    // 'ery good
                    break;
                } else {
                    // fuck again
                    // TODO ditto
                }
            }

            reviseColor();

            double error = robot.opticalDistanceSensor.getLightDetected() - LINE_FOLLOW_TARGET;
            // We still need to move forward.
            if (robot.opticalDistanceSensor.getLightDetected() > LINE_FOLLOW_TARGET) {
                // The input from the light sensors overrides the phone camera
                // Sorry, Phone Camera. You were crappy anyways.
                robot.rightMotor.setPower(speed + error * P_LINE_COEFF);
                robot.leftMotor.setPower(speed - error * P_LINE_COEFF);
                continue;
            } else {
                double gyroError = robot.gyro.getHeading() - target;

                if (gyroError > 0) {
                    robot.rightMotor.setPower(speed + error * P_LINE_COEFF);
                    robot.leftMotor.setPower(speed - error * P_LINE_COEFF);
                }
                else {
                    robot.rightMotor.setPower(speed + error * P_LINE_COEFF);
                    robot.leftMotor.setPower(speed - error * P_LINE_COEFF);
                }
                continue;
            }
        }
    }

    public boolean reviseColor () {
        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(false);
            } else {
                robot.push(true);
            }
            return true;
        } else if (robot.colorSensor.blue() > robot.colorSensor.red()) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.push(true);
            } else {
                robot.push(false);
            }
            return true;
        }

        return false;
    }
    //endregion

    //region gyroutils
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
        robotError = targetAngle - gyro.getIntegratedZValue();
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
    //endregion
}
