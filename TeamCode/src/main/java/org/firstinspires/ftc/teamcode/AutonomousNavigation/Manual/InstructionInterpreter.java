package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotHardware.Robot;

/**
 * Created by hsunx on 10/21/2016.
 */

public class InstructionInterpreter {

    Robot robot;

    public Instruction[] instructionSet;
    public Instruction currentInstruction;

    int currentInstructionIndex;

    public void SetInstructions (Instruction[] instructions) {
        instructionSet = instructions;
        currentInstructionIndex = 0;
        currentInstruction = instructionSet[currentInstructionIndex];
    }

    public InstructionInterpreter () {
        robot = Robot.robot;
        if (robot == null)
            RobotLog.e("ah dong. The robot instance doesn't exist (have you made sure to instantiate it first?)");
    }

    // Ideally, run this every "loop" during the opmode
    public void ExecuteInstruction () {
        if (!currentInstruction.initialized) {
            currentInstruction.Init();
            return;
        }
        if (currentInstruction.finished) {
            // Run the next instruction
            currentInstructionIndex++;
            currentInstruction = instructionSet[currentInstructionIndex];
            return;
        }
        // Loop
        currentInstruction.Loop();
    }
}
