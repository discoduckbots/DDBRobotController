package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    private DcMotor liftMotor;
    private DcMotor pivotMotor;

    public Arm(DcMotor liftMotor, DcMotor pivotMotor) {

        this.liftMotor = liftMotor;
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.pivotMotor = pivotMotor;
        this.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getLiftPos() {
        return liftMotor.getCurrentPosition();
    }
    public int getPivotPos() {
        return pivotMotor.getCurrentPosition();
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

    public void pivotForward(double power) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setPower(power);
    }

    public void pivotBackward(double power) {
        pivotMotor.setDirection(DcMotor.Direction.REVERSE);
        pivotMotor.setPower(power);
    }

    public void stopPivot() {
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setPower(0);
    }

    public void pivotToPosition(int position, double power) {
        pivotMotor.setTargetPosition(position);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(power);

    }

}