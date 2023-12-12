package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelGrabber {
    private Servo grabberServo;

    //uncomment
    //private Servo leftIntakeGrabber;
    //private Servo rightIntakeGrabber;
    //private Servo leftHookGrabber;
    //private Servo rightHookGrabber;
    //private DcMotor intakeFlip;
    private Servo wristServo;
    //comment out grabberServo and wristServo

    //private int UP_POSITION = 0;
    private boolean isClosed = true;
    private boolean buttonPress = false;
    private boolean isClosedPivot = true;
    private boolean buttonPressPivot = false;

    /*
    //Change class name to PixelMechanism
    public PixelMechanism(DcMotor intakeFlip, Servo rightHookGrabber, Servo leftHookGrabber, Servo rightIntakeGrabber, Servo leftIntakeGrabber) {
        this.intakePivot = intakePivot;
        this.rightHookGrabber = rightHookGrabber;
        this.leftHookGrabber = leftHookGrabber;
        this.rightIntakeGrabber = rightIntakeGrabber;
        this.leftIntakeGrabber = rightIntakeGrabber;
    }

     */
    public PixelGrabber(Servo grabberServo, Servo wristServo) {
        this.grabberServo = grabberServo;
        this.wristServo = wristServo;
    }
    //private Servo pusherServo;
    public void grab() {
        grabberServo.setPosition(0.5);
    }
    public void release() {
        grabberServo.setPosition(1.0);
    }

    /*
       public void flipToPosition(int position, double power) {
        intakePivot.setTargetPosition(position);
        intakePivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakePivot.setPower(power);

    }
        public void intakeLeft(int position) {
            leftIntakeGrabber(setPosition(position));
        }

        public void intakeRight(int position) {
            rightIntakeGrabber(setPosition(position));
        }
        public void hookLeft(int position) {
            rightHookGrabber(setPosition(position));
        }
        public void hookRight(int position) {
            leftHookGrabber(setPosition(position));
        }

        public void grabFlipHook() {
            leftIntakeGrabber.intakeLeft(0);
            rightIntakeGrabber.intakeRight(0);
            //maybe sleep(250);
            intakeFlip.flipToPostition(UP_POSITION);
            //maybe sleep(250);
            rightHookGrabber.hookRight(0);
            leftHookGrabber.hookLeft(0);
        }

     */

    public void onPress() {
        if (buttonPress) return;
        buttonPress = true;
        if (isClosed) {
            isClosed = false;
            release();
        }
        else {
            isClosed = true;
            grab();
        }
    }
    public void onRelease() {
        buttonPress = false;
    }

    public double getWristPostion(){
        return wristServo.getPosition();
    }

    public void onPressPivot() {
        if (buttonPressPivot) return;
        buttonPressPivot = true;
        if (isClosedPivot) {
            isClosedPivot = false;
            rotate(0.0);
        }
        else {
            isClosedPivot = true;
            rotate(1.0);
        }
    }
    public void onReleasePivot() {
        buttonPressPivot = false;
    }

    public void rotate(double position) {
        if (position >= 0.0 && position <= 1.0){
            wristServo.setPosition(position);
        }
    }

    public void pushPixel() {
        //pusherServo.setPosition(1.0);
    }

}
