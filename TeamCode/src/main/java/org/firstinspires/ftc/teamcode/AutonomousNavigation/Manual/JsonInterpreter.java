package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

/**
 * Created by hsunx on 10/21/2016.
 */

import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.Map;

public class JsonInterpreter {
    public Instruction FromString (String string) {
        Gson gson = new Gson();
        Type type = new TypeToken<Map<String, String>>(){}.getType();
        Map<String, String> myMap = gson.fromJson(string, type);
        return FromString(myMap);
    }

    // Do include the file extension
    public Instruction[] FromTxtFile (String fileInFIRSTDirectory) {
        StringBuilder text = new StringBuilder();
        BufferedReader reader;
        try {
            File sdCard = Environment.getExternalStorageDirectory();

            File file = new File(sdCard + "/FIRST/" + fileInFIRSTDirectory);
            reader = new BufferedReader(new FileReader(file));
            String line;
            while ((line = reader.readLine()) != null) {
                text.append(line);
                text.append(" ");
            }
            reader.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        // Oh god this is such a horrible hack. Ah well - less than a month to competition
        String[] instructionStrings = text.toString().split("---");
        Instruction[] instructions = new Instruction[instructionStrings.length];

        for (int i = 0; i < instructionStrings.length; i++) {
            instructions[i] = FromString(instructionStrings[i]);
        }

        return instructions;
    }

    public Instruction FromString (Map <String, String> map) {
        if (!map.containsKey("type")) {
            RobotLog.e("Well shit. JSON Interpreter failed, no type argument found");
            return null;
        }
        // All has gone well
        Instruction instruction = Instructions.valueOf(map.get("type")).instruction;
        instruction.FromMap(map);
        return instruction;
    }

    public enum Instructions {
        Movement (new MovementInstruction());

        final Instruction instruction;
        Instructions (Instruction instruction) {
            this.instruction = instruction;
        }
    }
}
