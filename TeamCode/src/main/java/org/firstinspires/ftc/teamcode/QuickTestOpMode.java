package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoEx;

/**
 * Created by hsunx on 10/29/2016.
 */
@Autonomous (name = "Tester", group = "Test")
public class QuickTestOpMode extends OpMode {
    ServoEx extendedServo;
    Servo servo;
    boolean clockwise = true;
    double servoSpeed = 0.02;
    double servoPosition;
    @Override
    public void init() {
        ServoEx servo = hardwareMap.get(ServoEx.class, "servo");
        if (servo == null) {
            // Make sure something crashes
            int x = 0/0;
        }
        this.servo = hardwareMap.servo.get("servo");
        extendedServo.setPwmRange(new ServoEx.ServoPwmRange(553, 2425));
        extendedServo.setPwmEnable();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        if (clockwise) {
            if (servo.getPosition() >= 0.99999) {
                clockwise = false;
                return;
            }
            servoPosition += servoSpeed;
            servo.setPosition(servoPosition);
        }
        else {
            if (servo.getPosition() <= 0.000001) {
                clockwise = true;
                return;
            }
            servoPosition -= servoSpeed;
            servo.setPosition(servoPosition);
        }
        telemetry.addData("servo pos", servo.getPosition());
    }

    @Override
    public void stop() {
        super.stop();
    }
}
