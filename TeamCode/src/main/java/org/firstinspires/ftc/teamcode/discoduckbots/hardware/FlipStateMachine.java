package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FlipStateMachine {
    public enum State {
        IDLE,
        FLIP_UP,
        FLIP_DOWN
    }

    private State currentState = State.IDLE;

    public State onFlipMovement(DcMotor flipMotor) {
        if (currentState == State.FLIP_UP && !flipMotor.isBusy()) {
            currentState = State.IDLE;
            return State.FLIP_UP;
        }
        else if (currentState == State.FLIP_DOWN && !flipMotor.isBusy()) {
            currentState = State.IDLE;
            return State.FLIP_DOWN;
        }
        return null;
    }

    public void flippingUp() {
        currentState = State.FLIP_UP;
    }

    public void flippingDown() {
        currentState = State.FLIP_DOWN;
    }

}
