package org.firstinspires.ftc.teamcode.RobotHardware;

import android.os.DropBoxManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.AbstractMap;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Created by hsunx on 10/28/2016.
 */

public class Robot {

    // There should only be one instance of this (singleton)
    public static Robot robot;

    // Telemetry
    public Telemetry telemetry;

    // Elapsed time
    public ElapsedTime timer  = new ElapsedTime();

    // Hardware map
    public HardwareMap hwMap;

    public boolean initialized;

    // TODO add "arm releasers"

    // region
    // Modern Robotic Gyro
    public ModernRoboticsI2cGyro gyro;

    public OpticalDistanceSensor opticalDistanceSensor;

    public ModernRoboticsI2cColorSensor colorSensor;
    // endregion



    // region
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

    //endregion

    // region
    // Scissorlift Servos
    public Servo leftScissorliftServo;
    public Servo rightScissorliftServo;

    // Button pushing servos
    public Servo leftPushServo;
    public Servo rightPushServo;

    public Servo conveyorGate;
    public Servo armReleasers;
    // endregion

    // Some consts
    public final double noPushLeft = 0;
    public final double noPushRight = 230/255;
    public final double shootSpeed = 0.75;

    public Robot () {
        // Set static instance to this. The other one will live on in memory and then get GC'ed
        robot = this;
    }

    public void init (HardwareMap hardwareMap, Telemetry telemetry) {
        hwMap = hardwareMap;
        this.telemetry = telemetry;


        // Initiate motors and servos
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShootMotor = hardwareMap.dcMotor.get("left_shoot");
        rightShootMotor = hardwareMap.dcMotor.get("right_shoot");

        leftShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyorMotor = hardwareMap.dcMotor.get("conveyor");

        ballCollectionMotor = hardwareMap.dcMotor.get("ball_collector");

        scissorliftMotor = hardwareMap.dcMotor.get("scissor_lift");

        scissorliftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftScissorliftServo = hardwareMap.servo.get("left_scissorlift_servo");
        rightScissorliftServo = hardwareMap.servo.get("right_scissorlift_servo");

        leftPushServo = hardwareMap.servo.get("left_push");
        rightPushServo = hardwareMap.servo.get("right_push");

        conveyorGate = hardwareMap.servo.get("gate");

        armReleasers = hardwareMap.servo.get("arm_releasers");
        // Init sensors
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        opticalDistanceSensor = (hardwareMap.opticalDistanceSensor.get("ods"));

        colorSensor = ((ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color_sensor"));

        initialized = true;

    }
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) timer.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        timer.reset();
    }

    public void shoot (boolean shooting) {
        if (initialized) {
            robot.leftShootMotor.setPower(shooting ? shootSpeed : 0);
            robot.rightShootMotor.setPower(shooting ? shootSpeed : 0);
        }
    }
}
