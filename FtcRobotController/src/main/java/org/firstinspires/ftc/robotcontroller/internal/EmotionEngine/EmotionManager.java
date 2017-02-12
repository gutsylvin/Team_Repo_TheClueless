package org.firstinspires.ftc.robotcontroller.internal.EmotionEngine;

import android.content.Context;
import android.speech.tts.TextToSpeech;

import java.util.Locale;

/**
 * Created by hsunx on 1/20/2017.
 */

public class EmotionManager {
    static TextToSpeech textToSpeech;
    static boolean initialized;
    public static void speak(String text) {
        if (initialized) {
            textToSpeech.speak(text, TextToSpeech.QUEUE_FLUSH, null);
        }
    }

    public static void initializeTTS(Context context) {
        textToSpeech = new TextToSpeech(context, new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                textToSpeech.setLanguage(Locale.US);
                initialized = true;
            }
        });
    }
}
