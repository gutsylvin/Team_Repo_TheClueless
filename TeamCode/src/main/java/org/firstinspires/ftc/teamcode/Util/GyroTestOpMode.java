package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;
import org.lasarobotics.library.sensor.modernrobotics.Voltage;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * Created by hsunx on 10/21/2016.
 */
@Autonomous(name = "Util: Gyro Test", group = "Util")
public class GyroTestOpMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ArrayList<GyroSensor> gyros;

    @Override
    public void init() {

    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("heading", Robot.robot.gyro.getHeading());
    }
}
