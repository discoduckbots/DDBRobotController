package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class PixelGrabber {
    private Servo grabberServo;
    private Servo wristServo;
    private boolean isClosed = true;
    private boolean buttonPress = false;
    private boolean isClosedPivot = true;
    private boolean buttonPressPivot = false;

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
