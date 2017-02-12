package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions;

import com.google.gson.annotations.Expose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual.Instructions.Instruction;

/**
 * Created by hsunx on 11/5/2016.
 */

public class ServoInstruction extends Instruction {

    @Expose
    public String servo_name;

    @Expose
    public double speed;

    @Expose
    public double position;

    boolean clockwise;

    // This assumes that you are using the HS-485HB servo
    public final double ENDPOINT = 230/255;

    public Servo servo;

    @Override
    public void Init() {
        super.Init();
        if (position > ENDPOINT) {
            // You're going past the endpoint of the servo (well not exactly, but close)
            RobotLog.e("Servo instruction " + name + " has a position set past the mechanical endpoint of the servo");
            Finished();
        }
        servo = robot.hwMap.get(Servo.class, servo_name);
        clockwise = position > servo.getPosition();
    }

    @Override
    public void Loop() {
        double servoPosition = servo.getPosition();
        servoPosition += speed * (clockwise ? 1 : -1);
        if (servoPosition > ENDPOINT) {
            servo.setPosition(ENDPOINT);
            Finished();
        }
        if (servoPosition < 0) {
            servo.setPosition(0);
            Finished();
        }
        if (clockwise && servoPosition >= position) {
            servo.setPosition(position);
            Finished();
        }
        if (!clockwise && servoPosition <= position) {
            servo.setPosition(position);
            Finished();
        }
        servo.setPosition(position);
    }

}
