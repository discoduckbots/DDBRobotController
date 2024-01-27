package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;

    //public DcMotor extensionMotor;
    //delete pivotMotor and rename to extensionMotor
    public static double LIFT_POWER = 1.0;
    public static int LIFT_AUTO_POS = -608;
    public static int LIFT_2_AUTO_POS = 631;
    public static int AUTOEXTEND = 1933;
    public static int LIFT_ROW1 = 445;
    public static int LIFT_ROW2 = 1097;
    public static int LIFT_ROW3 = 1676;
    public static int EXTEND_IN = 285;
    public static int EXTEND_OUT = -1584;
    private static int LIFT_ROW4;
    private static int LIFT_ROW5;
    private boolean buttonPressArm = false;
    private boolean isRow0 = false;
    private boolean isRow1 = false;
    private boolean isRow2 = false;
    private boolean isRow3 = false;

    public Arm(DcMotor liftMotor1, DcMotor liftMotor2) {

        this.liftMotor1 = liftMotor1;
        this.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.liftMotor2 = liftMotor2;
        this.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getLiftPos() {
        return liftMotor1.getCurrentPosition();
    }

    public int getLiftPos2() {
        return liftMotor2.getCurrentPosition();
    }

    public void lift(double power) {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setPower(power);
        liftMotor2.setPower(-power);
        Log.d("LIFT ", "pos1: " + liftMotor1.getCurrentPosition() + "pos2: " + liftMotor2.getCurrentPosition());
    }

    public void lower(double power) {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setPower(-power);
        liftMotor2.setPower(power);
    }

    public void stopLift() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
    }

    public void liftToPosition(int position, int position2, double power) {
        liftMotor1.setTargetPosition(position);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setTargetPosition(position2);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }


    public void onReleaseArm() {
        buttonPressArm = false;
    }
   /* public void onPressArm() {
        //if (buttonPressArm) return;
        buttonPressArm = true;
        if(getLiftPos() == 0) {
            liftToPosition(LIFT_ROW1, LIFT_POWER);
        } else if (getLiftPos() == LIFT_ROW1) {
            liftToPosition(LIFT_ROW2, LIFT_POWER);
        } else if (getLiftPos() == LIFT_ROW2) {
            liftToPosition(LIFT_ROW3, LIFT_POWER);
        }
    } */

    public void print() {
        Log.d("LIFT_MOTOR:" , "pos : " + liftMotor1.getCurrentPosition());
        //Log.d("EXT_MOTOR:" , "pos : " + extensionMotor.getCurrentPosition());
    }
}