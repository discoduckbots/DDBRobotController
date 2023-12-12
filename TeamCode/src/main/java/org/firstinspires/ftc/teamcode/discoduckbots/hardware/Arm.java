package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    private DcMotor liftMotor;

    private DcMotor extensionMotor;
    //delete pivotMotor and rename to extensionMotor


    public Arm(DcMotor liftMotor, DcMotor extensionMotor) {

        this.liftMotor = liftMotor;
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor = extensionMotor;
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor.setPower(power);
    }

    public void extendBackward(double power) {
        extensionMotor.setDirection(DcMotor.Direction.REVERSE);
        extensionMotor.setPower(power);
    }

    public void stopExtend() {
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setPower(0);
    }

    public void extendToPosition(int position, double power) {
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);

    }

}