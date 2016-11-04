package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

/**
 * Created by hsunx on 10/21/2016.
 */

import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.List;

public class JsonInterpreter {
    final String TYPE_INSTRUCTION_SEPERATOR = "###";

    private Instruction[] FromString (String string) {
        // Initialize all dat json stuffs
        GsonBuilder gsonBuilder = new GsonBuilder();
        gsonBuilder.excludeFieldsWithoutExposeAnnotation();
        Gson gson = gsonBuilder.create();

        String[] temp = string.split(TYPE_INSTRUCTION_SEPERATOR);
        String types = temp[0];
        String instructions = temp[1];

        Type typeStringsType = new TypeToken<List<String>>(){}.getType();
        List<String> typeStrings = gson.fromJson(types, typeStringsType);

        for (String type : typeStrings) {
            Instructions instructionType = Instructions.valueOf(type);
            Class<? extends Instruction> instruction = instructionType.instruction.getClass();
        }
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
        } catch (IOException e) {
            e.printStackTrace();
        }

        return FromString(text.toString());
    }

    public enum Instructions {
        movement (new MovementInstruction());

        final Instruction instruction;

        Instructions (Instruction instruction) {
            this.instruction = instruction;
        }
    }
}
