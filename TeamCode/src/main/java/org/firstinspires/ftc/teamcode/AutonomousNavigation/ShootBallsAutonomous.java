package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.firstinspires.ftc.teamcode.ShooterCalibration;


/**
 * Created by hsunx on 10/22/2016.
 */

@Autonomous(name = "Autonomous: Shoot Balls Autonomous", group = "Autonomous")
public class ShootBallsAutonomous extends LinearOpMode {
    /* Declare OpMode members. */
    Robot robot = new Robot();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double TURN_SPEED = 0.2;     // Nominal half speed for better accuracy.
    static final double TURN_TIMEOUT = 10;

    static final double START_ENCODER = 200;    // Driving subroutines will use this encoder value as the "revving up" period
    static final double END_ENCODER = 500;

    static final double HEADING_THRESHOLD = 2;      // (NOT) As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.06;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.20;     // Larger is more responsive, but also less stable
    static final double P_LINE_COEFF = 0.15;

    boolean left1;
    boolean left2;


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap, telemetry);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        while (gyro.getHeading() != 0) {
            gyro.calibrate();

            while (gyro.isCalibrating()) {
                Thread.sleep(50);
                idle();
            }
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        robot.gyro.resetDeviceConfigurationForOpMode();
        robot.gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..

        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData("Team Color", MatchDetails.color);
            telemetry.addData("right servo position", robot.rightPushServo.getPosition());
            telemetry.update();
            idle();
        }


        robot.conveyorGate.setPosition(0.863);
        prepareShoot(true);

        robot.armReleasers.setPosition(0);

        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961); // 245/255

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn


        shoot();
        prepareShoot(false);

        /*
        int startLeftTurn;
        int startRightTurn;
        int endLeftTurn;
        int endRightTurn;


        if (MatchDetails.color == MatchDetails.TeamColor.RED) {

            encoderDrive(0.25, 0.75, 0.25, -400, -400, 5000);

            Thread.sleep(125);
            gyroTurn(TURN_SPEED * 0.9, -133);

            encoderDrive(0.25, 0.75, 0.25, 3000, 2900, 8000);

            gyroDriveUntilLine(0.15, 0.45);

            gyroTurn(TURN_SPEED * 0.5, -93);
            //encoderDrive(0.35, 325, 300, 3000);
            pushBeacon(0.45, -90);
            encoderDrive(0.7, -300, -300, 3000);
            gyroTurn(TURN_SPEED * 0.75, -178);
            encoderDrive(0.5, 0.75, 0.5, 2175, 2150, 10000);
            gyroDriveUntilLine(0.15, 0.45);
            encoderDrive(0.20, -50, -50, 2000);
            gyroTurn(TURN_SPEED * 0.5, -93);
            encoderDrive(0.8, 500, 500, 1000);
            pushBeacon(0.5, -90);
        }
        else if (MatchDetails.color == MatchDetails.TeamColor.BLUE) {

            encoderDrive(0.25, 0.75, 0.25, -625, -625, 5000);

            Thread.sleep(125);
            gyroTurn(TURN_SPEED * 0.5, 125);

            encoderDrive(0.25, 0.75, 0.25, 2800, 2850, 8000);

            gyroDriveUntilLine(0.15, 0.45);

            gyroTurn(TURN_SPEED * 0.5, 93);
            //encoderDrive(0.35, 325, 300, 3000);
            pushBeacon(0.45, 90);
            encoderDrive(0.7, -450, -450, 3000);
            gyroTurn(TURN_SPEED * 0.75, 175);
            encoderDrive(0.5, 0.75, 0.5, 2400, 2140, 10000);
            gyroDriveUntilLine(0.15, 0.45);
            // For some reason this isn't needed??? Lol ok that's cool too.
            encoderDrive(0.20, 50, 50, 2000);
            gyroTurn(TURN_SPEED * 0.5, 95);
            encoderDrive(0.8, 300, 300, 1000);
            pushBeacon(0.5, 90);

        }
        */
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    void prepareShoot(boolean shooting) throws InterruptedException {
        robot.shoot(shooting, ShooterCalibration.SHOOTING_SPEED);
        Thread.sleep(2000);
    }

    void shoot() throws InterruptedException {
        robot.conveyorMotor.setPower(1);
        robot.ballCollectionMotor.setPower(1);
        Thread.sleep(4000);
        robot.conveyorMotor.setPower(0);
        robot.ballCollectionMotor.setPower(0);
    }

    // positive turn for clockwise, negative for counterclockwise.
    public void localTurn (double speed, double angle) throws InterruptedException{
        int newLeftTarget;
        int newRightTarget;

        boolean clockwise = (angle - robot.gyro.getHeading() < 0);

        robot.leftMotor.setPower(clockwise ? speed : -speed);
        robot.rightMotor.setPower(clockwise ? -speed : speed);

        while (opModeIsActive() && robot.gyro.getHeading() < angle) {
            telemetry.addData("Gyro", gyro.getHeading());
            telemetry.update();
            idle();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double left, double right,
                             double timeoutS) throws InterruptedException {
        encoderDrive(speed, speed, speed, left, right, timeoutS);
    }

    public void encoderDrive (double speed, double counts, double timeoutMs) throws InterruptedException {
        encoderDrive(speed, speed, speed, counts, counts, timeoutMs / 1000);
    }

    public void encoderDrive(double speed,
                             double speed1,
                             double speed2,
                             double left, double right,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int encoderStartLeft;
        int encoderStartRight;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            encoderStartLeft = robot.leftMotor.getCurrentPosition();
            encoderStartRight = robot.rightMotor.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            newLeftTarget = encoderStartLeft + (int) (left);
            newRightTarget = encoderStartRight + ((int) (right));

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
                    (runtime.seconds() < timeoutS) &&
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

    void pushBeacon(double speed, double angle) throws InterruptedException {

        MatchDetails.TeamColor colorDetected = MatchDetails.TeamColor.RED; // This will be properly assigned later so that's cool

        Thread.sleep(250);

        while (robot.colorSensor.red() == robot.colorSensor.blue()) {
            encoderDrive(0.25, 150, 150, 1000);
            idle();
            // huh. um. well we're screwed, but not really
        }
        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.rightPushServo.setPosition(0.5);
                colorDetected = MatchDetails.TeamColor.RED;
            } else {
                robot.leftPushServo.setPosition(0.431);
                colorDetected = MatchDetails.TeamColor.BLUE;
            }
        } else if (robot.colorSensor.blue() > robot.colorSensor.red()) {
            if (MatchDetails.color == MatchDetails.TeamColor.RED) {
                robot.leftPushServo.setPosition(0.431);
                colorDetected = MatchDetails.TeamColor.RED;
            } else {
                robot.rightPushServo.setPosition(0.5);
                colorDetected = MatchDetails.TeamColor.BLUE;
            }
        }


        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("COLOR DETECTED", colorDetected.toString());
        telemetry.addData("Team Color", MatchDetails.color);
        Thread.sleep(500);

        encoderDrive(speed, 1000, 2000);

        /*while (true) {
            telemetry.addData("color", colorDetected.toString());
            telemetry.update();
            if (colorDetected == MatchDetails.TeamColor.RED) {
                int reading = robot.colorSensor.red();
                if (reading == 16) {
                    break;
                }
                robot.leftMotor.setPower(speed);
                robot.rightMotor.setPower(speed);
            } else {
                int reading = robot.colorSensor.blue();
                if (reading == 16) {
                    break;
                }
                robot.leftMotor.setPower(speed);
                robot.rightMotor.setPower(speed);
            }
            idle();
        }*/
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961);
    }

    void playSoundFile() {

    }

    /**
     * Method to follow a line of the specified reflectivity. Suggested that you
     * calibrate the Optical Distance Sensor first
     *
     * @param speed    Target speed for motion.
     * @param distance Distance to move in encoder counts
     * @param target   Target reflectivity for the Optical Distance Sensor. Between 0.0 and 1.0
     */

    public void followLine(double speed,
                           double distance,
                           double target) throws InterruptedException {

        if (opModeIsActive()) {
            int moveCount = (int) (distance * COUNTS_PER_INCH);
            while (opModeIsActive() && robot.leftMotor.getCurrentPosition() < moveCount && robot.rightMotor.getCurrentPosition() < moveCount) {
                double error = (target - robot.opticalDistanceSensor.getLightDetected()) * P_LINE_COEFF;

                double leftPower = speed;
                double rightPower = speed;

                if (error < 0) {
                    leftPower -= error;
                } else {
                    rightPower -= error;
                }

                robot.leftMotor.setPower(leftPower);
                robot.rightMotor.setPower(rightPower);

                telemetry.addData("following", "line");
                telemetry.addData("ods value", robot.opticalDistanceSensor.getLightDetected());
                telemetry.addData("error", error);
                telemetry.addData("left power", leftPower);
                telemetry.addData("right power", rightPower);

                idle();
            }
        }

    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward. wtf why
     *                 If a relative angle is required, add/subtract from current heading.
     */

    public void gyroDrive(double speed,
                          double distance,
                          double angle) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double startTime = getRuntime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance /*\*COUNTS_PER_INCH*/);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            /// / start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
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
        }
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param speed2   Target speed for long distances.
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward. wtf why
     *                 If a relative angle is required, add/subtract from current heading.
     */

    public void gyroDriveModified(double speed,
                                  double speed2,
                                  double speed3,
                                  double distance,
                                  double angle) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        int encoderStartLeft;
        int encoderStartRight;
        double startTime = getRuntime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance /*\*COUNTS_PER_INCH*/);
            encoderStartLeft = robot.leftMotor.getCurrentPosition();
            encoderStartRight = robot.rightMotor.getCurrentPosition();
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            /// / start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                boolean pastStart = (Math.abs(robot.leftMotor.getCurrentPosition() - encoderStartLeft) > START_ENCODER
                        || Math.abs(robot.rightMotor.getCurrentPosition() - encoderStartRight) > START_ENCODER);

                boolean pastEnd = (Math.abs(newLeftTarget - robot.leftMotor.getCurrentPosition()) > END_ENCODER
                        || Math.abs(newRightTarget - robot.rightMotor.getCurrentPosition()) > END_ENCODER);

                leftSpeed = /*(pastEnd ? speed3 : */(pastStart ? speed2 : speed) - steer;
                rightSpeed = /*(pastEnd ? speed3 : */(pastStart ? speed2 : speed) + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
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
        }
    }

    public void gyroDriveUntilLine(double speed
                            /*double angle*/, double target) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // start motion.
            speed = Range.clip(speed, -1.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive()) {
                double scaledValue = robot.opticalDistanceSensor.getLightDetected();
                telemetry.addData("light", scaledValue);
                telemetry.addData("gyro", robot.gyro.getHeading());
                telemetry.update();
                if (scaledValue >= target) {
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    return;
                }

                // Allow time for other processes to run.
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
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold(double speed, double angle, double holdTime)
            throws InterruptedException {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
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
            rightSpeed = speed * steer;
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
}
