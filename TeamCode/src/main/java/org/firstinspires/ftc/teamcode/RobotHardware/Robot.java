package org.firstinspires.ftc.teamcode.RobotHardware;

import android.os.DropBoxManager;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensorImpl;
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
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.library.drive.ScaledDcMotor;
import org.lasarobotics.library.util.MathUtil;
import org.lasarobotics.vision.android.Util;

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
    public AdafruitI2cColorSensor adafruitColorSensor;

    public VoltageSensor voltageSensor;
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

    public ScaledDcMotor scissorLiftArmMotor;

    //endregion

    // region
    // Button pushing servos
    public Servo leftPushServo;
    public Servo rightPushServo;

    public Servo conveyorGate;
    public Servo armReleasers;
    // endregion

    public final double shootSpeed = 0.43;

    public Robot () {
        // Set static instance to this. The other one will live on in memory and then get GC'ed
        robot = this;
    }

    public void noPush () {
        robot.leftPushServo.setPosition(0);
        robot.rightPushServo.setPosition(0.961);
    }

    public void push (boolean left) {
        if (left) {
            robot.leftPushServo.setPosition(0.431);
            robot.rightPushServo.setPosition(0.961);
        }
        else {
            robot.rightPushServo.setPosition(0.5);
            robot.leftPushServo.setPosition(0);
        }
    }

    public void pushBoth (boolean push) {
        if (push) {
            robot.leftPushServo.setPosition(0.431);
            robot.rightPushServo.setPosition(0.5);
        }
        else {
            robot.leftPushServo.setPosition(0);
            robot.rightPushServo.setPosition(0.961);
        }
    }

    public void init (HardwareMap hardwareMap, Telemetry telemetry) {
        hwMap = hardwareMap;
        this.telemetry = telemetry;


        // Initiate motors and servos
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShootMotor = hardwareMap.dcMotor.get("left_shoot");
        rightShootMotor = hardwareMap.dcMotor.get("right_shoot");

        leftShootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        conveyorMotor = hardwareMap.dcMotor.get("conveyor");

        ballCollectionMotor = hardwareMap.dcMotor.get("ball_collector");

        scissorliftMotor = hardwareMap.dcMotor.get("scissor_lift");

        scissorliftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scissorLiftArmMotor = new ScaledDcMotor(hardwareMap.dcMotor.get("scissor_lift_arms"));
        scissorLiftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissorLiftArmMotor.setMinMax(-0.5, 0.5);

        leftPushServo = hardwareMap.servo.get("left_push");
        rightPushServo = hardwareMap.servo.get("right_push");

        conveyorGate = hardwareMap.servo.get("gate");

        armReleasers = hardwareMap.servo.get("arm_releasers");
        // Init sensors
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        opticalDistanceSensor = (hardwareMap.opticalDistanceSensor.get("ods"));

        colorSensor = ((ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color_sensor"));
        //adafruitColorSensor = ((AdafruitI2cColorSensor)hardwareMap.colorSensor.get("color_sensor_adafruit"));

        voltageSensor = hardwareMap.voltageSensor.get("Shoot Motors");

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

    public void shoot (boolean shooting, double speed) {
        if (initialized) {
            robot.leftShootMotor.setPower(shooting ? speed : 0);
            robot.rightShootMotor.setPower(shooting ? speed : 0);
        }
    }
}