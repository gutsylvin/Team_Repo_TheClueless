package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

/**
 * Created by hsunx on 10/21/2016.
 */

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonParser;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

public class JsonInterpreter {
    final String TYPE_INSTRUCTION_SEPERATOR = "###";

    private Instruction[] FromString (String string) {
        // Initialize all dat json stuffs
        GsonBuilder gsonBuilder = new GsonBuilder();
        gsonBuilder.excludeFieldsWithoutExposeAnnotation();
        Gson gson = gsonBuilder.create();

        String[] temp = string.split(TYPE_INSTRUCTION_SEPERATOR);
        RobotLog.d(string);
        String types = temp[0];
        String instructions = temp[1];

        Type typeStringsType = new TypeToken<List<String>>(){}.getType();
        List<String> typeStrings = gson.fromJson(types, typeStringsType);

        ArrayList<Class<? extends Instruction>> instructionClasses = new ArrayList<>();
        ArrayList<Instruction> instructionInstances = new ArrayList<>();

        JsonParser parser = new JsonParser();

        for (String type : typeStrings) {
            Instructions instructionType;
            try {
                instructionType = Instructions.valueOf(type);
            }
            catch (IllegalArgumentException i) {
                RobotLog.e("Type string invalid: " + type);
                continue;
            }
            Class<? extends Instruction> instruction = instructionType.instruction;
            instructionClasses.add(instruction);
        }

        JsonArray array = parser.parse(instructions).getAsJsonArray();

        // Check to see if the array length is the same as the type array length
        if (typeStrings.size() != array.size()) {
            // ABORT!
            RobotLog.e("The size of the type strings list isn't equal to the size of the instructions list!");
            return null;
        }

        for (int i = 0; i < typeStrings.size(); i++) {
            try {
                Instruction newInstruction = gson.fromJson(array.get(i), instructionClasses.get(i));
                RobotLog.d(newInstruction.toString());
                instructionInstances.add(newInstruction);
            }
            catch (Exception e) {
                RobotLog.e("something failed during JSON deserialization");
            }
        }

        // If all goes well (pretty unlikely)
        Instruction[] returnArray = new Instruction[instructionInstances.size()];
        returnArray = instructionInstances.toArray(returnArray);
        return returnArray;
    }

    // Do include the file extension
    public Instruction[] FromTxtFile (String fileInFIRSTDirectory) {
        StringBuilder text = new StringBuilder();
        BufferedReader reader;
        try {
            // ZTE why
            File file = new File("/storage/sdcard1/FIRST/" + fileInFIRSTDirectory);
            reader = new BufferedReader(new FileReader(file));
            String line;
            while ((line = reader.readLine()) != null) {
                text.append(line);
                text.append(" ");
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return FromString(text.toString());
    }

    public enum Instructions {
        movement (MovementInstruction.class),
        turn (TurnInstruction.class);

        final Class<? extends Instruction> instruction;

        Instructions (Class<? extends Instruction> instruction) {
            this.instruction = instruction;
        }
    }
}
