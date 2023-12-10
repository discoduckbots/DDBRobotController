package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public class DuckSensorTensorFlow {


    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private static float LEFT_POSITION = 100.0f;
    private static float RIGHT_POSITION = 60.0f;
    private static float CENTER_MIN = 350.0f;
    private static float CENTER_MAX = 400.0f;


    private static String PINK_DUCK = "PDUCK";
    private static String BLUE_DUCK = "BDUCK";

    private static final String DUCK_FILE = "duckfile1.tflite";

    public DuckSensorTensorFlow (HardwareMap hardwareMap ){
        this.hardwareMap= hardwareMap;
        initTfod();
    }
    public int getDuckPos() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        Log.d("DUCK_POS", "POS: " + tfod.getRecognitions());


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            Log.d("DUCK_POS", "LABEL: " + recognition.getLabel() + " " + recognition.getConfidence());
            Log.d("DUCK_POS", "left_border: " + recognition.getLeft());

            if ((recognition.getLabel().equals(PINK_DUCK ) || recognition.getLabel().equals(BLUE_DUCK ))&& recognition.getConfidence() >= 90) {
               if (recognition.getLeft() < LEFT_POSITION) {
                   return 1;
               }
              /* if (RIGHT_POSITION == recognition.getLeft())  {
                  return 2;
              } */
              if (recognition.getLeft() > CENTER_MIN && recognition.getLeft() < CENTER_MAX) {
                  return 2;
              }
            }

        }   // end for() loop
        return 0;
    }

    private void initTfod() {
        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();
        List<String> labels = new ArrayList<>();
        labels.add (PINK_DUCK);
        labels.add (BLUE_DUCK);
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(DUCK_FILE)
                //.setModelFileName(DUCK_FILE)
                //TODO change file name later


                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(labels)
               .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true) ~~
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        /* TfodProcessor.Builder builder = new TfodProcessor.Builder();
        builder = builder.setModelFileName(TFOD_MODEL_FILE);
        builder = builder.setModelLabels(BLUE_DUCK);
        builder = builder.setIsModelTensorFlow2(true) ;
        TfodProcessor tfod = builder.build();
*/
        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

    }   // end method initTfod()

}
