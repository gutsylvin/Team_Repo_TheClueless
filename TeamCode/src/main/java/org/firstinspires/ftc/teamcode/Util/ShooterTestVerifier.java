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

    enum SauceType {
        LINEAR,
        POLYNOMIAL,
        NO_OUTLIERS_LINEAR,
        NO_OUTLIERS_POLYNOMIAL
    }

    SauceType type;

    Gamepad previousGamepad1;
    boolean shooting;

    boolean selected = false;

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
        if (gamepad1.a)
            type = SauceType.LINEAR;
        if (gamepad1.b)
            type = SauceType.NO_OUTLIERS_LINEAR;
        if (gamepad1.x)
            type = SauceType.POLYNOMIAL;
        if (gamepad1.y)
            type = SauceType.NO_OUTLIERS_POLYNOMIAL;

        if (gamepad1.start)
            selected = true;

        if (!selected)
            return;

        double voltage = sensor.getVoltage();

        if (gamepad1.a && !previousGamepad1.a) {
            shooting = !shooting;
            if (shooting) {
                switch (type) {
                    case LINEAR:
                        shootSpeed = -0.076052 * voltage + 1.4659;
                        break;
                    case NO_OUTLIERS_LINEAR:
                        shootSpeed = -0.030539 * voltage + 1.2671;
                        break;
                    case POLYNOMIAL:
                        shootSpeed = -117.11 * (Math.pow(0.84863, voltage)) + -2.3759 * voltage + 45.242;
                        break;
                    case NO_OUTLIERS_POLYNOMIAL:
                        shootSpeed = -0.15829 * (Math.pow(voltage, 3)) + 5.9856 * (Math.pow(voltage, 2)) + -75.445 * voltage + 317.47;
                        break;
                    default:

                }
            }
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
        telemetry.addData("shoot power", shootSpeed);
        telemetry.update();
    }
}
