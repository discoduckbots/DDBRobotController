package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    public Servo droneServo;
    public static double LAUNCH_POS;

    public DroneLauncher(Servo droneServo) {
        this.droneServo = droneServo;
    }

    public void launch() {
        droneServo.setPosition(LAUNCH_POS);
    }
}
