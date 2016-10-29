package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by hsunx on 10/28/2016.
 */
@Autonomous (name = "Color Sensor Test", group = "Test")
public class ColorSensorTestOpMode extends OpMode {
    ModernRoboticsI2cColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("color_sensor");
        colorSensor.enableLed(false);
    }

    @Override
    public void loop() {
        colorSensor.enableLed(false);
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void start() {
        super.start();
    }
}
