package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.discoduckbots.opmode.teleop.NewMecanumDrivetrainTeleOp;

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

    public static int FLIP_DOWN = -115;
    public static int FLIP_UP_TELEOP = 112;
    private static double FLIP_POWER = .75;
    public static double LG_OPEN_POS = 1;
    public static double LG_CLOSE_POS = 0;
    public static double LG_HALF_POS = .5;
    public static double RG_OPEN_POS = 0;
    public static double RG_CLOSE_POS = 1;
    public static double RG_HALF_POS = .5;
    public static double LH_OPEN_POS = 0;
    public static double LH_CLOSE_POS = 1;
    public static double RH_OPEN_POS = 1;
    public static double RH_CLOSE_POS = 0;
    private static int FLIP_UP = 0;



    public PixelMechanism(DcMotor flipMotor, Servo rightHook, Servo leftHook, Servo rightGrabber, Servo leftGrabber, FlipStateMachine flipStateMachine) {
        this.flipMotor = flipMotor;
       // flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeLeft(LG_CLOSE_POS);
            intakeRight(RG_CLOSE_POS);
            //maybe sleep(250);
            flipToPosition(0, FLIP_POWER);
            flipStateMachine.flippingUp();
            //maybe sleep(250);
            hookLeft(LH_CLOSE_POS);
            hookRight(RH_CLOSE_POS);
        }

        public void newGrabFlipHook(LinearOpMode opmode) {
            flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeLeft(LG_CLOSE_POS);
            intakeRight(RG_CLOSE_POS);
            opmode.sleep(200);
            flipToPosition(FLIP_UP_TELEOP, FLIP_POWER);
        }
        public void resetIntake(LinearOpMode opmode) {
            flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeLeft(LG_CLOSE_POS);
            intakeRight(RG_CLOSE_POS);
            flipToPosition(0, FLIP_POWER);
            opmode.sleep(200);
            intakeLeft(1);
            intakeRight(0);
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
        flipToPosition(FLIP_DOWN, FLIP_POWER);
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

    public void onPressLeftHook() {
        if (buttonPressLH) return;
        buttonPressLH = true;
        if (isClosedLH) {
            isClosedLH = false;
            hookLeft(LH_OPEN_POS);
        }
        else {
            isClosedLH = true;
            hookLeft(LH_CLOSE_POS);
        }
    }
    public void onReleaseLeftHook() {

        buttonPressLH = false;
    }

    public void onPressRightHook() {
        if (buttonPressRH) return;
        buttonPressRH = true;
        if (isClosedRH) {
            isClosedRH = false;
            hookRight(RH_OPEN_POS);
        }
        else {
            isClosedRH = true;
            hookRight(RH_CLOSE_POS);
        }
    }
    public void onReleaseRightHook() {

        buttonPressRH = false;
    }
    public void print() {
        Log.d("FLIP_MOTOR:" , "pos : " + flipMotor.getCurrentPosition());
    }



   /* public double getWristPostion(){
        return wristServo.getPosition();
    } */



}
