package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    private DcMotor liftMotor;

    private DcMotor extensionMotor;
    //delete pivotMotor and rename to extensionMotor
    private static double LIFT_POWER = .5;
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

    public Arm(DcMotor liftMotor, DcMotor extensionMotor) {

        this.liftMotor = liftMotor;
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor = extensionMotor;
        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getLiftPos() {
        return liftMotor.getCurrentPosition();
    }
    public int getExtensionPos() {
        return extensionMotor.getCurrentPosition();
    }
    public void lift(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setPower(power);
        Log.d("LIFT ", "pos: " + liftMotor.getCurrentPosition());
    }

    public void lower(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setPower(-power);
    }

    public void stopLift() {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(0);
    }

    public void liftToPosition(int position, double power) {
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
    }

    public void extendForward(double power) {
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setPower(power);
    }

    public void extendBackward(double power) {
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setPower(power * -1);
    }

    public void stopExtend() {
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setPower(0);
    }

    public void extendToPosition(int position, double power) {
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setTargetPosition(position);
        extensionMotor.setPower(power);

    }
    public void onReleaseArm() {
        buttonPressArm = false;
    }
    public void onPressArm() {
        //if (buttonPressArm) return;
        buttonPressArm = true;
        if(getLiftPos() == 0) {
            liftToPosition(LIFT_ROW1, LIFT_POWER);
        } else if (getLiftPos() == LIFT_ROW1) {
            liftToPosition(LIFT_ROW2, LIFT_POWER);
        } else if (getLiftPos() == LIFT_ROW2) {
            liftToPosition(LIFT_ROW3, LIFT_POWER);
        }
    }

    public void print() {
        Log.d("LIFT_MOTOR:" , "pos : " + liftMotor.getCurrentPosition());
        Log.d("EXT_MOTOR:" , "pos : " + extensionMotor.getCurrentPosition());
    }
}