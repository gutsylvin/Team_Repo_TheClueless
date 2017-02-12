package org.lasarobotics.library.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Created by hsunx on 12/30/2016.
 */

public class ScaledDcMotor implements DcMotor{
    double min = -1;
    double max = 1;

    DcMotor dcMotor;

    public DcMotor getDcMotor() {
        return dcMotor;
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public Direction getDirection() {
        return dcMotor.getDirection();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public double getPower() {
        return dcMotor.getPower();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    @Override
    public int getMaxSpeed() {
        return dcMotor.getMaxSpeed();
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public int getVersion() {
        return dcMotor.getVersion();
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotor.getManufacturer();
    }

    @Override
    public RunMode getMode() {
        return dcMotor.getMode();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotor.getConnectionInfo();
    }

    @Override
    public String getDeviceName() {
        return dcMotor.getDeviceName();
    }

    @Override
    public void close() {
        dcMotor.close();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void setDirection(Direction direction) {
        dcMotor.setDirection(direction);
    }

    @Override
    public void setMaxSpeed(int encoderTicksPerSecond) {
        dcMotor.setMaxSpeed(encoderTicksPerSecond);
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotor.setMode(mode);
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotor.setTargetPosition(position);
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    public ScaledDcMotor(DcMotor dcMotor){
        this.dcMotor = dcMotor;
    }

    public void setMinMax (double min, double max) {
        this.min = min;
        this.max = max;
    }

    @Override
    public void setPower(double power) {
        dcMotor.setPower(scaleNumber(power, min, max));
    }

    double scaleNumber (double number, double min, double max) {
        // http://stackoverflow.com/questions/5294955/how-to-scale-down-a-range-of-numbers-with-a-known-min-and-max-value
        if (max <= min)
            throw new ArithmeticException("Max is less than or equal to min -- undefined");

        return ((max - min) * (number - min)) / (max - min) + min;
    }
}
