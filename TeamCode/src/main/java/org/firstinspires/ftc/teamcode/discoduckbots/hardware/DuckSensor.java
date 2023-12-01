package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DuckSensor {
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    double SENSOR1_MAX = 5;
    double SENSOR2_MAX = 5;

    public DuckSensor(DistanceSensor distanceSensor1, DistanceSensor distanceSensor2) {

        this.distanceSensor1 = distanceSensor1;
        this.distanceSensor2 = distanceSensor2;
    }

    private boolean sensor1Detected(double distance) {
        return distance < SENSOR1_MAX;
    }
    private boolean sensor2Detected(double distance) {
        return distance < SENSOR2_MAX;
    }

    public double getDistance1() {
        return distanceSensor1.getDistance(DistanceUnit.CM);
    }

    public double getDistance2() {
        return distanceSensor2.getDistance(DistanceUnit.CM);
    }

    public int getDuckPos() {
        double distance1 = distanceSensor1.getDistance(DistanceUnit.CM);
        double distance2 = distanceSensor2.getDistance(DistanceUnit.CM);
        Log.d("FTC-Duck" , "d1: " + distance1 + " d2: " + distance2);
        if(sensor1Detected(distance1) == true && sensor2Detected(distance2) == false){
            return 1;
        } else if (sensor1Detected(distance1) == false && sensor2Detected(distance2)== true) {
            return 3;
        } else return 2;
    }

}