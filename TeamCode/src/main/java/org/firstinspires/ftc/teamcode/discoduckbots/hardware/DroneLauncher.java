package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    public Servo droneServo;
    public Servo holdServo;
    public static double LAUNCH_POS = 0.0;
    public static double HOLD_POS = 0.0;

    public DroneLauncher(Servo droneServo, Servo holdServo) {
        this.droneServo = droneServo;
        this.holdServo = holdServo;
    }

    public void launch() {
        droneServo.setPosition(LAUNCH_POS);
    }

    public void release() {
        holdServo.setPosition(HOLD_POS);
    }
}
