package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LegacyModulePortDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by hsunx on 12/17/2016.
 */

@Autonomous (name = "secret sauce", group = "potato")
public class ShooterTestVerifier extends OpMode {
    Robot robot;
    VoltageSensor sensor;

    Gamepad previousGamepad1;
    boolean shooting;

    double shootSpeed;
    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, telemetry);
        sensor = hardwareMap.voltageSensor.get("Shoot Motors");
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !previousGamepad1.a) {
            shooting = !shooting;
            shootSpeed = -0.076052 * sensor.getVoltage() + 1.4659;
        }

        if (shooting) {
            robot.shoot(true, shootSpeed);
            robot.conveyorMotor.setPower(1);
        }
        else {
            robot.conveyorMotor.setPower(0);
            robot.shoot(false, 0);
        }

        try {
            previousGamepad1.copy(gamepad1);
        }
        catch (RobotCoreException e) {
            // Swallow the exception
        }
        telemetry.addData("voltage", sensor.getVoltage());
        telemetry.update();
    }
}
