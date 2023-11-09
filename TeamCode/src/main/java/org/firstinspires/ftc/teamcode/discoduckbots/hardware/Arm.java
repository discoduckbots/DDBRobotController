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
        this.pivotMotor = pivotMotor;
        this.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getLiftPos() {
        return liftMotor.getCurrentPosition();
    }
    private int getPivotPos() {
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
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setPower(power);
    }

    public void stopLift() {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(0);
    }

    public void liftToPosition(int position, double power) {
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);

//        while (liftMotor.isBusy()) {
//            //wait for motor to be done
//        }
    }

    public void pivotForward(double power) {
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

        while (pivotMotor.isBusy()) {
            //wait for motor to be done
        }
    }

}