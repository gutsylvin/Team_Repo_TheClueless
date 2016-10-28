package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        telemetry.addData("Status", "Initialized");

        gyros = new ArrayList<>();
        Iterator<GyroSensor> iterator = hardwareMap.gyroSensor.iterator();
        for (int i = 0; i < hardwareMap.gyroSensor.size(); i++) {
            gyros.add(iterator.next());
        }
        for (GyroSensor gyro: gyros
             ) {
            gyro.calibrate();

        }
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
        for (GyroSensor gyro: gyros) {
            if (gyro.isCalibrating())
                continue;
            telemetry.addData("X axis - " + gyro.getDeviceName(), gyro.rawX());
            telemetry.addData("Y axis - " + gyro.getDeviceName(), gyro.rawY());
            telemetry.addData("Z axis - " + gyro.getDeviceName(), gyro.rawZ());
            telemetry.addData("Heading - " + gyro.getDeviceName(), gyro.getHeading());
        }
    }
}
