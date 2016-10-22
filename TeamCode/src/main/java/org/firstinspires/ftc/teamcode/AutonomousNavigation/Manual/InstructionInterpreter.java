package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hsunx on 10/21/2016.
 */

public abstract class InstructionInterpreter {
    DcMotor leftDrive;
    DcMotor rightDrive;

    boolean finished;

    MovementInstruction[] instructionSet;

    public void SetInstructions (MovementInstruction[] instructions) {
        instructionSet = instructions;
    }

    public InstructionInterpreter (DcMotor left, DcMotor right) {
        leftDrive = left;
        rightDrive = right;
    }

    public abstract boolean Finished ();
    public abstract void ExecuteInstruction ();
}
