package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotHardware.Flywheels;
import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.lasarobotics.library.util.Log;

/**
 * Created by hsunx on 12/16/2016.
 */

@TeleOp (name = "Shooter Tester", group = "ayy kek lmao")
public class ShooterTester extends OpMode {
    Robot robot = new Robot();

    public static double shootSpeed;
    double startEncoderLeft;
    double startEncoderRight;
    double startTime;

    Flywheels flywheels;

    Log logger;

    boolean shooting;
    ElapsedTime timer;

    Gamepad previousGamepad1;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        previousGamepad1 = new Gamepad();
        flywheels = new Flywheels();
        timer = new ElapsedTime();
        logger = new Log("FIRST", "Shooter_test_log");
        telemetry.update();
    }

    @Override
    public void loop() {

        double leftDiff = 0;
        double rightDiff = 0;
        double totalTime = 0;

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            shootSpeed += 0.01;
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            shootSpeed -= 0.01;
        }

        if (gamepad1.a && !previousGamepad1.a) {
            shooting = !shooting;
            if (shooting) {
                robot.shoot(true, shootSpeed);
                startEncoderLeft = robot.leftMotor.getCurrentPosition();
                startEncoderRight = robot.rightMotor.getCurrentPosition();
                startTime = timer.time();
            }
            else {
                robot.shoot(false);
                leftDiff = robot.leftMotor.getCurrentPosition() - startEncoderLeft;
                rightDiff = robot.rightMotor.getCurrentPosition() - startEncoderRight;
                totalTime = timer.time() - startTime;
                logger.add("Left Difference", String.valueOf(leftDiff));
                logger.add("Right Difference", String.valueOf(rightDiff));
                logger.add("Time", String.valueOf(totalTime));
                logger.add("Speed", String.valueOf(shootSpeed));
            }
        }

        telemetry.addData("left difference", leftDiff);
        telemetry.addData("right difference", rightDiff);
        telemetry.addData("time", time);
        telemetry.addData("speed", shootSpeed);

        boolean conveyor = gamepad1.b;
        boolean sweeper = gamepad1.x;

        robot.ballCollectionMotor.setPower(sweeper ? 1 : 0);
        robot.conveyorMotor.setPower(conveyor ? 1 : 0);

        try {
            previousGamepad1.copy(gamepad1);
        }
        catch (RobotCoreException e) {

        }

    }

    @Override
    public void stop() {
        logger.saveAs(Log.FileType.CSV);
        super.stop();
    }
}
