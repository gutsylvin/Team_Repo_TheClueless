package org.firstinspires.ftc.teamcode.AutonomousNavigation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by hsunx on 12/1/2016.
 */

public class OpticalDistanceSensorCalibrationOpMode extends OpMode {
    public static double white = 0;
    public static double black = 1;

    public OpticalDistanceSensor ods;

    public static double scaleValueForOds (double value) {
        return Range.clip((white - black) * (value), 0, 1);
    }

    @Override
    public void init() {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            white = ods.getLightDetected();
        }
        if (gamepad1.b) {
            black = ods.getLightDetected();
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}
