package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

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

import java.util.List;

public class DuckSensorTensorFlow {


    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private static float LEFT_POSITION = 20.0f;
    private static float RIGHT_POSITION = 60.0f;
    private static float CENTER_POSITION = 40.0f;


    private static String PINK_DUCK = "PinkDuck";
    private static String BLUE_DUCK = "BlueDuck";

    public DuckSensorTensorFlow (HardwareMap hardwareMap ){
        this.hardwareMap= hardwareMap;
        initTfod();
    }
    private int getDuckPos() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            if ((recognition.getLabel().equals(PINK_DUCK ) || recognition.getLabel().equals(BLUE_DUCK ))&& recognition.getConfidence() >= 90) {
               if (LEFT_POSITION == recognition.getLeft()) {
                   return 1;
               }
              if (RIGHT_POSITION == recognition.getLeft())  {
                  return 2;
              }
              if (CENTER_POSITION == recognition.getLeft()){
                  return 3;
              }
            }

        }   // end for() loop
        return 0;
    }

    private void initTfod() {
        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(BLUE_DUCK);
                .setIsModelTensorFlow2(true) ;
                //.setIsModelQuantized(true) ~~
                //.setModelInputSize(300) 
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        TfodProcessor.Builder builder = new TfodProcessor.Builder();
        builder = builder.setModelFileName(TFOD_MODEL_FILE);
        builder = builder.setModelLabels(BLUE_DUCK);
        builder = builder.setIsModelTensorFlow2(true) ;
        TfodProcessor tfod = builder.build();

        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);



    }   // end method initTfod()

}
