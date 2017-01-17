package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

import static org.firstinspires.ftc.teamcode.Util.ShooterTester.shootSpeed;

@TeleOp(name="Encoder Test", group="Test")
public class EncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor testMotor = null;
    DcMotor testMotorRight = null;

    @Override
    public void runOpMode() {
        testMotor = Robot.robot.leftShootMotor;
        testMotorRight = Robot.robot.rightShootMotor;

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted()) {

        }
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        // run motor for 10 seconds
        Robot.robot.conveyorMotor.setPower(1);
        testMotor.setPower(shootSpeed);
        testMotorRight.setPower(shootSpeed);
        while (opModeIsActive() && (runtime.time() < 10)) {
            telemetry.addData("Encoder left", "%4.1f:  %d counts", runtime.time(), testMotor.getCurrentPosition());
            telemetry.addData("Encoder right", "%4.1f:  %d counts", runtime.time(), testMotorRight.getCurrentPosition());
            telemetry.update();
        }
        Robot.robot.conveyorMotor.setPower(0);
        testMotor.setPower(0.0);
        testMotorRight.setPower(0.0);

        telemetry.addData("Encoder left", "%5.0f Counts Per Second", (double)(testMotor.getCurrentPosition()) / runtime.time());
        telemetry.addData("Encoder right", "%5.0f Counts Per Second", (double)(testMotorRight.getCurrentPosition()) / runtime.time());

        telemetry.update();
        while (opModeIsActive());

    }
}