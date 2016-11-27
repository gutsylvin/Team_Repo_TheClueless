package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;

/**
 * Created by hsunx on 10/29/2016.
 */
@Autonomous (name = "Servo Tester", group = "Test")
public class QuickTestOpMode extends OpMode {

    ModernRoboticsDigitalTouchSensor touchSensor;
    ModernRoboticsI2cColorSensor colorSensor;
    ModernRoboticsAnalogOpticalDistanceSensor ods;
    Servo servo;

    double servoPosition;
    double servoSpeed = 0.02;

    boolean clockwise = true;

    @Override
    public void init() {
        // colorSensor = ((ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color_sensor"));
        // touchSensor = ((ModernRoboticsDigitalTouchSensor) hardwareMap.touchSensor.get("touch_sensor"));
        // ods = ((ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods"));

        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        // colorSensor.enableLed(touchSensor.isPressed());
        // telemetry.addData("touch", touchSensor.isPressed());
        // telemetry.addData("red", colorSensor.red());
        // telemetry.addData("green", colorSensor.green());
        // telemetry.addData("blue", colorSensor.blue());
        // telemetry.addData("alpha", colorSensor.alpha());
        // telemetry.addData("ods reflect", ods.getLightDetected() + ", raw: " + ods.getRawLightDetected());

        if (clockwise) {
            if (servo.getPosition() >= 1) {
                clockwise = false;
            }
            else {
                servoPosition += servoSpeed;
                servo.setPosition(servoPosition);
            }
        }
        else {
            if (servo.getPosition() <= 0) {
                clockwise = true;
            }
            else {
                servoPosition -= servoSpeed;
                servo.setPosition(servoPosition);
            }
        }

    }

    @Override
    public void stop() {
        super.stop();
    }
}
