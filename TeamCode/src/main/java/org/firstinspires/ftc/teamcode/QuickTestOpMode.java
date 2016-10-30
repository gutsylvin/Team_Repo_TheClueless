package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

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
        Servo servo = hardwareMap.servo.get("servo");
        if (servo instanceof ServoEx) {
            extendedServo = (ServoEx) servo;
            this.servo = servo;
            extendedServo.setPwmRange(new ServoEx.ServoPwmRange(553, 2425));
            extendedServo.setPwmEnable();
        }
        else {
            RobotLog.e("shit");
            this.servo = servo;
            this.servo.scaleRange(0, 1.3);
        }
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
