package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hsunx on 10/29/2016.
 */
@Autonomous (name = "Servo Tester", group = "Test")
public class QuickTestOpMode extends OpMode {

    Servo servo;

    boolean clockwise = true;
    double servoSpeed = 0.02;
    double servoPosition;

    double mechanicalEndpoint = 1.0;
    @Override
    public void init() {
        this.servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        if (servoPosition + gamepad1.left_stick_y * servoSpeed < mechanicalEndpoint) {
            servo.setPosition(servoPosition + gamepad1.left_stick_y * servoSpeed);
        }
        mechanicalEndpoint += gamepad1.left_trigger * servoSpeed;
        telemetry.addData("servo pos", servo.getPosition());
        telemetry.addData("endpoint", mechanicalEndpoint);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
