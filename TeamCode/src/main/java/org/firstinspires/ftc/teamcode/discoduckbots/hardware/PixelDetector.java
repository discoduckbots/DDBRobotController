package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PixelDetector {

    private NormalizedColorSensor leftColorSensor;
    private NormalizedColorSensor rightColorSensor;

    private static final int RANGE = 40; //mm of range

    public PixelDetector(NormalizedColorSensor leftColorSensor, NormalizedColorSensor rightColorSensor){
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        /*lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");*/
    }

    private double getDistance (NormalizedColorSensor colorSensor){
        DistanceSensor distanceSensor = (DistanceSensor) colorSensor;
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    private void detectColors(){
        final float[] leftHsvValues = new float[3];
        final float[] rightHsvValues = new float[3];

        NormalizedRGBA leftColors = leftColorSensor.getNormalizedColors();
        Color.colorToHSV(leftColors.toColor(), leftHsvValues);

        NormalizedRGBA rightColors = rightColorSensor.getNormalizedColors();
        Color.colorToHSV(rightColors.toColor(), rightHsvValues);
    }

    public boolean isBothPixels(){
        return isLeftPixel() && isRightPixel();
    }

    public boolean isLeftPixel(){
        return getDistance(leftColorSensor) < RANGE;
    }

    public boolean isRightPixel(){
        return getDistance(rightColorSensor) < RANGE;
    }


}
