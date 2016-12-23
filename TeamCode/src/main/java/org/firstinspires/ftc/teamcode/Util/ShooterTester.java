package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by hsunx on 12/16/2016.
 */

@TeleOp (name = "Shooter Tester", group = "ayy kek lmao")
public class ShooterTester extends OpMode {
    Robot robot = new Robot();

    double shootSpeed;

    Gamepad previousGamepad1;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            shootSpeed += 0.02;
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            shootSpeed -= 0.02;
        }
        telemetry.addData("speed", shootSpeed);
        telemetry.update();

        boolean shooting = gamepad1.a;
        boolean conveyor = gamepad1.b;

        robot.conveyorMotor.setPower(conveyor ? 1 : 0);
        robot.shoot(shooting, shootSpeed);

        try {
            previousGamepad1.copy(gamepad1);
        }
        catch (RobotCoreException e) {

        }
    }
}
