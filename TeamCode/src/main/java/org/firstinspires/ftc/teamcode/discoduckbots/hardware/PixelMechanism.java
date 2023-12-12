package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.StateMachine;

public class PixelMechanism {
    public Servo leftGrabber;
    public Servo rightGrabber;
    public Servo leftHook;
    public Servo rightHook;
    public DcMotor flipMotor;
    public FlipStateMachine flipStateMachine;

    //private int UP_POSITION = 0;
    private boolean isClosedLG = true;
    private boolean isClosedRG = true;
    private boolean isClosedLH = true;
    private boolean isClosedRH = true;
    private boolean buttonPressLG = false;
    private boolean buttonPressRG = false;
    private boolean buttonPressLH = false;
    private boolean buttonPressRH = false;

    private static int DOWN_POSITION = -122;
    private static int FLIP_POWER;
    private static double LG_OPEN_POS = 0.4;
    private static double LG_CLOSE_POS = 0.15;
    private static double RG_OPEN_POS = 0.35;
    private static double RG_CLOSE_POS = 0.6;
    private static double LH_OPEN_POS;
    private static double LH_CLOSE_POS;
    private static double RH_OPEN_POS;
    private static double RH_CLOSE_POS;



    public PixelMechanism(DcMotor flipMotor, Servo rightHook, Servo leftHook, Servo rightGrabber, Servo leftGrabber, FlipStateMachine flipStateMachine) {
        this.flipMotor = flipMotor;
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightHook = rightHook;
        this.leftHook = leftHook;
        this.rightGrabber = rightGrabber;
        this.leftGrabber = leftGrabber;
        this.flipStateMachine = flipStateMachine;

    }

       public void flipToPosition(int position, double power) {
        flipMotor.setTargetPosition(position);
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipMotor.setPower(power);

    }

        public void intakeLeft(double position) {
            leftGrabber.setPosition(position);
        }
        public void intakeRight(double position) {
            rightGrabber.setPosition(position);
        }
        public void hookLeft(double position) {
            leftHook.setPosition(position);
        }
        public void hookRight(double position) {
            rightHook.setPosition(position);
        }

        public void grabFlipHook() {
            intakeLeft(LG_CLOSE_POS);
            intakeRight(RG_CLOSE_POS);
            //maybe sleep(250);
            flipToPosition(0, FLIP_POWER);
            flipStateMachine.flippingUp();
            //maybe sleep(250);
        }

        public void updateState() {
            FlipStateMachine.State state = flipStateMachine.onFlipMovement(flipMotor);
            if (state == FlipStateMachine.State.FLIP_UP) {
                hookRight(0);
                hookLeft(0);
                //maybe sleep(250);
                intakeLeft(LG_OPEN_POS);
                intakeRight(RG_OPEN_POS);
            }
        }

        public void dropPixels() {
        hookRight(RH_OPEN_POS);
        hookLeft(LH_OPEN_POS);
        flipToPosition(DOWN_POSITION, FLIP_POWER);
        }

        public void increasePosition(Servo servo, String servoName) {
            servo.setPosition(servo.getPosition() + 0.01);
            Log.d(servoName, "POS: " + servo.getPosition());
        }

    public void decreasePosition(Servo servo, String servoName) {
        servo.setPosition(servo.getPosition() - 0.01);
        Log.d(servoName, "POS: " + servo.getPosition());
    }

    public void onPressLeftGrabber() {
        if (buttonPressLG) return;
        buttonPressLG = true;
        if (isClosedLG) {
            isClosedLG = false;
            intakeLeft(LG_OPEN_POS);
        }
        else {
            isClosedLG = true;
            intakeLeft(LG_CLOSE_POS);
        }
    }
    public void onReleaseLeftGrabber() {
        buttonPressLG = false;
    }

    public void onPressRightGrabber() {
        if (buttonPressRG) return;
        buttonPressRG = true;
        if (isClosedRG) {
            isClosedRG = false;
            intakeRight(RG_OPEN_POS);
        }
        else {
            isClosedRG = true;
            intakeRight(RG_CLOSE_POS);
        }
    }
    public void onReleaseRightGrabber() {
        buttonPressRG = false;
    }

   /* public double getWristPostion(){
        return wristServo.getPosition();
    } */



}
