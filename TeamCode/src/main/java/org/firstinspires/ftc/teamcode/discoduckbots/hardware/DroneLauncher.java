package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    public Servo droneServo;
    //public Servo holdServo;
    public static double LAUNCH_POS = 1.0;
    public static double HOLD_POS = 0.0;

    public DroneLauncher(Servo droneServo) {
        this.droneServo = droneServo;
    }

    public void launch() {

        droneServo.setPosition(LAUNCH_POS);
    }
}
