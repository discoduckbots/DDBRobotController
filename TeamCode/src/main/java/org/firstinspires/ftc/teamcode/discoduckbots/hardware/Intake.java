package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    private DcMotor intakeMotor;
    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void intake(double power) {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(power);
    }

    public void outtake(double power) {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setPower(power);
    }
    public void stop() {
        intakeMotor.setPower(0.0);
    }
}
