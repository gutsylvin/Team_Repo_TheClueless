package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import static org.firstinspires.ftc.teamcode.Util.ShooterTester.shootSpeed;

@TeleOp(name="Encoder Test", group="Test")
public class EncoderTest extends LinearOpMode {

    final int DELAY_SETTLE = 1000;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor testMotorLeft = null;
    DcMotor testMotorRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        testMotorLeft = Robot.robot.leftShootMotor;
        testMotorRight = Robot.robot.rightShootMotor;

        testMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted()) {

        }
        testMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // run motor for 10 seconds
        Robot.robot.conveyorMotor.setPower(1);
        testMotorLeft.setPower(shootSpeed);
        testMotorRight.setPower(shootSpeed);
        Thread.sleep(DELAY_SETTLE);

        runtime.reset();
        double previousEncoderValueLeft = testMotorLeft.getCurrentPosition();
        double previousEncoderValueRight = testMotorRight.getCurrentPosition();

        while (opModeIsActive() && (runtime.time() < 6)) {
            telemetry.addData("Encoder left", "%4.1f:  %d counts", runtime.time(), testMotorLeft.getCurrentPosition());
            telemetry.addData("Encoder right", "%4.1f:  %d counts", runtime.time(), testMotorRight.getCurrentPosition());
            telemetry.update();
        }
        Robot.robot.conveyorMotor.setPower(0);
        testMotorLeft.setPower(0.0);
        testMotorRight.setPower(0.0);

        telemetry.addData("Encoder left", "%5.0f Counts Per Second", (double)(testMotorLeft.getCurrentPosition() - previousEncoderValueLeft ) / runtime.time());
        telemetry.addData("Encoder right", "%5.0f Counts Per Second", (double)(testMotorRight.getCurrentPosition() - previousEncoderValueRight ) / runtime.time());

        telemetry.update();
        while (opModeIsActive());

    }
}