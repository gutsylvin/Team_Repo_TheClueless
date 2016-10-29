package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by hsunx on 10/28/2016.
 */

public class Robot {

    // There should only be one instance of this (singleton)
    public static Robot robot;

    // Modern Robotic Gyro
    public ModernRoboticsI2cGyro gyro;

    // Telemetry
    public Telemetry telemetry;

    // Drive
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    // Shooting
    public DcMotor leftShootMotor;
    public DcMotor rightShootMotor;

    // Conveyor
    public DcMotor conveyorMotor;

    // Ball Collection
    public DcMotor ballCollectionMotor;

    // Scissorlift
    public DcMotor scissorliftMotor;

    // Scissorlift Servos
    public Servo leftScissorliftServo;
    public Servo rightScissorliftServo;

    // Button pushing servos
    public Servo leftPushServo;
    public Servo rightPushServo;

    // Elapsed time
    private ElapsedTime period  = new ElapsedTime();

    // Constants for button pusher
    public final float noPush = 0f;
    public final float push = 0.5f;

    // Hardware map
    HardwareMap hwMap;

    public Robot () {
        // Set static instance to this. The other one will live on in memory and then get GC'ed
        robot = this;
    }

    public void init (HardwareMap hardwareMap, Telemetry telemetry) {
        hwMap = hardwareMap;
        this.telemetry = telemetry;

        //HardwareMap.DeviceMapping<Servo> servos = hwMap.servo;

        // Initiate motors and servos
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftShootMotor = dcMotors.get("left_shoot");
//        rightShootMotor = dcMotors.get("right_shoot");
//
//        conveyorMotor = dcMotors.get("conveyor");
//
//        ballCollectionMotor = dcMotors.get("ball_collector");
//
        scissorliftMotor = hardwareMap.dcMotor.get("scissor_lift");

        scissorliftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftScissorliftServo = hardwareMap.servo.get("scissor_left_servo");
        rightScissorliftServo = hardwareMap.servo.get("scissor_right_servo");

        leftPushServo = hardwareMap.servo.get("left_push");
        rightPushServo = hardwareMap.servo.get("right_push");
//
//        leftPushServo.setPosition(noPush);
//        rightPushServo.setPosition(noPush);

        // Init sensors
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

    }
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
