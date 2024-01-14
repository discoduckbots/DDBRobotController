package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class PixelMechanism {
    public Servo leftGrabber;
    public Servo rightGrabber;
    public DcMotor flipMotor;
    public DcMotor pivotMotor;

    public TouchSensor pivotTouchSensor;

    public FlipStateMachine flipStateMachine;

    private boolean isInGrabbingPosition = false;
    private boolean isLeftGrabberClosed = false;
    private boolean isRightGrabberClosed = false;
    private boolean isClosedLG = true;
    private boolean isClosedRG = true;
    private boolean buttonPressLG = false;
    private boolean buttonPressRG = false;

    public static final double FLIP_POWER = 0.5;
    private static final double PIVOT_POWER = 0.5;
    public static final int FLIP_SCORE = -281;
    public static final int FLIP_GRAB = 832;
    public static final int PIVOT_SCORE = -1218;
    public static final int PIVOT_GRAB = 0;
    public static final double LG_OPEN_POS = 1;
    public static final double LG_CLOSE_POS = 0;
    public static final double RG_OPEN_POS = 0;
    public static final double RG_CLOSE_POS = 1;

    public PixelMechanism(DcMotor flipMotor, DcMotor pivotMotor, Servo rightGrabber, Servo leftGrabber, TouchSensor pivotTouchSensor, FlipStateMachine flipStateMachine) {
        this.flipMotor = flipMotor;
        this.pivotMotor = pivotMotor;
        this.rightGrabber = rightGrabber;
        this.leftGrabber = leftGrabber;
        this.pivotTouchSensor = pivotTouchSensor;
        this.flipStateMachine = flipStateMachine;

        this.flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void flipToPosition(int position, double power) {
        flipMotor.setTargetPosition(position);
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipMotor.setPower(power);
    }

    public void flipToGrabbingPosition(){
        flipToPosition(FLIP_GRAB, FLIP_POWER);
        isInGrabbingPosition = true;
    }

    public void flipToScoringPosition(){
        flipToPosition(FLIP_SCORE, FLIP_POWER);
        isInGrabbingPosition = false;
    }

    public void resetPivotEncoder(){
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void pivotToPosition(int position, double power) {
        if (pivotTouchSensor.isPressed()){
            resetPivotEncoder();
        }

        pivotMotor.setTargetPosition(position);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(power);
    }

    public void pivotToGrabbingPosition(LinearOpMode opMode) throws InterruptedException{
        if (!areGrabbersClosed()){
            closeGrabbers();
        }
        if (!isInGrabbingPosition){
            flipToGrabbingPosition();
            opMode.sleep(200);
        }
        pivotToPosition(PIVOT_GRAB, PIVOT_POWER);
    }

    public void pivotToScoringPosition(LinearOpMode opMode) throws InterruptedException{
        if (!areGrabbersClosed()){
            closeGrabbers();
        }
        if (!isInGrabbingPosition){
            flipToGrabbingPosition();
            opMode.sleep(200);
        }
        pivotToPosition(PIVOT_SCORE, PIVOT_POWER);
    }

    public boolean isPivotTouchSensorPressed(){
        return pivotTouchSensor.isPressed();
    }

    public void intakeLeft(double position) {
        leftGrabber.setPosition(position);
    }

    public void intakeRight(double position) {
        rightGrabber.setPosition(position);
    }

    public void toScore(LinearOpMode opmode) {
        try {
            pivotToScoringPosition(opmode);
            flipToScoringPosition();
        }
        catch (InterruptedException e){/*eat exception*/}
    }

    public void toGrab(LinearOpMode opmode) {
        try{
            flipToGrabbingPosition();
            pivotToGrabbingPosition(opmode);
        }
        catch (InterruptedException e){/*eat exception*/}
    }

    public void onePressGrabber() {
        if(leftGrabber.getPosition() == LG_OPEN_POS || rightGrabber.getPosition() == RG_OPEN_POS) {
           closeLeftGrabber();
           closeRightGrabber();
        }
        else {
            openLeftGrabber();
            openRightGrabber();
        }
    }

    public void onPressLeft() {
        if (leftGrabber.getPosition() == LG_OPEN_POS) {
            closeLeftGrabber();
        }
        else {
            openLeftGrabber();
        }
    }

    public void onPressRight() {
        if (rightGrabber.getPosition() == RG_OPEN_POS) {
            closeRightGrabber();
        }
        else {
            openRightGrabber();
        }
    }

    public void increasePosition(Servo servo, String servoName) {
        servo.setPosition(servo.getPosition() + 0.01);
        Log.d(servoName, "POS: " + servo.getPosition());
    }

    public void decreasePosition(Servo servo, String servoName) {
        servo.setPosition(servo.getPosition() - 0.01);
        Log.d(servoName, "POS: " + servo.getPosition());
    }

    public void print() {
        Log.d("FLIP_MOTOR:", "pos : " + flipMotor.getCurrentPosition());
        Log.d("FLIP_PIV", "flip pos : " + flipMotor.getCurrentPosition() +
                " piv pos : " + pivotMotor.getCurrentPosition());
    }

    public void openLeftGrabber(){
        leftGrabber.setPosition(LG_OPEN_POS);
        isLeftGrabberClosed = false;
    }

    public void closeLeftGrabber(){
        leftGrabber.setPosition(LG_CLOSE_POS);
        isLeftGrabberClosed = true;
    }

    public void openRightGrabber(){
        rightGrabber.setPosition(RG_OPEN_POS);
        isRightGrabberClosed = false;
    }

    public void closeRightGrabber(){
        rightGrabber.setPosition(RG_CLOSE_POS);
        isRightGrabberClosed = true;
    }

    private boolean areGrabbersClosed() {
        return isRightGrabberClosed && isLeftGrabberClosed;
    }

    public void closeGrabbers(){
        closeLeftGrabber();
        closeRightGrabber();
    }
}