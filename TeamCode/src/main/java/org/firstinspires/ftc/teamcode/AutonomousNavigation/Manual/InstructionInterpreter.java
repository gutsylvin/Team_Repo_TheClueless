package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hsunx on 10/21/2016.
 */

public class InstructionInterpreter {
    DcMotor leftDrive;
    DcMotor rightDrive;

    int leftEncoder, rightEncoder;
    int previousLeft, previousRight;

    boolean finished;

    MovementInstruction[] instructionSet;
    MovementInstruction currentInstruction;

    int currentInstructionIndex;

    public void SetInstructions (MovementInstruction[] instructions) {
        instructionSet = instructions;
        currentInstructionIndex = 0;
        currentInstruction = instructionSet[currentInstructionIndex];
    }

    public InstructionInterpreter (DcMotor left, DcMotor right) {
        leftDrive = left;
        rightDrive = right;
    }

    public void Finished () {

    }
    public void ExecuteInstruction () {

    }
}
