package org.firstinspires.ftc.teamcode.AutonomousNavigation.Manual;

/**
 * Created by hsunx on 10/21/2016.
 */
import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

public class JsonInterpreter {
    public MovementInstruction[] FromString (String string) {
        Gson gson = new Gson();
        List<MovementInstruction> movementInstructions = gson.fromJson(string, new TypeToken<List<MovementInstruction>>(){}.getType());
        MovementInstruction[] instructions = new MovementInstruction[movementInstructions.size()];
        return movementInstructions.toArray(instructions);
    }

    public MovementInstruction[] FromTxtFile (String fileOnSDCard) {
        StringBuilder text = new StringBuilder();
        BufferedReader reader;
        try {
            File sdCard = Environment.getExternalStorageDirectory();
            File file = new File (sdCard, fileOnSDCard);
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
        return FromString(text.toString());
    }
}
