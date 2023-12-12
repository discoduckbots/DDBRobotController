/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.discoduckbots.opmode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
/*
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Centerstage Opmode", group= "Linear Opmode")
@Disabled
/*
public class MecanumDrivetrainTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.75;
    private static double INTAKE_SPEED = 0.75;
    private static double OUTAKE_SPEED = 1.0;
    private static double SLIDE_SPEED = 0.75;
    private static double PIVOT_SPEED = 0.5;
    private static double HANG_SPEED = 0.3;
    private static int SLIDE_POS_1 = 0;
    private static int SLIDE_POS_2 = -350;
    private static int SLIDE_POS_3 = 100;
    private static int SLIDE_RESET = 0;
    private static int PIVOT_POS_1 = 2300;
    private static int PIVOT_POS_2 = 40;
    private static int PIVOT_POS_3 = 80;
    private static int PIVOT_RESET = 0;
    private int liftPosition = 0;
    private int pivotPosition = 0;
    boolean liftAtEncoderPos = false;
    boolean pivotAtEncoderPos = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;
    private Intake intake = null;
    private Arm arm = null;
    //private DcMotor hangMotor = null;
    private PixelGrabber pixelGrabber = null;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        arm = hardwareStore.getArm();
        intake = hardwareStore.getIntake();
        pixelGrabber = hardwareStore.getPixelGrabber();
        //hangMotor = hardwareStore.getHangMotor();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            Log.d("LIFT" , "pos : " + hardwareStore.liftMotor.getCurrentPosition());
            Log.d("PIVOT", "pos : " + hardwareStore.pivotMotor.getCurrentPosition());
            Log.d("ARM", "lift pos : " + hardwareStore.liftMotor.getCurrentPosition() +
                    "pivot pos : " + hardwareStore.pivotMotor.getCurrentPosition());

             //Gamepad 1
            mecanumDrivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, THROTTLE);

            /*if (gamepad1.left_trigger > 0.05) {
                intake.outtake(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0.05) {
                intake.intake(gamepad1.right_trigger);
            }
            else {
                intake.stop();
            }

            if (gamepad1.left_bumper) {
                intake.outtake(OUTAKE_SPEED);
            }
            else if (gamepad1.right_bumper) {
                intake.intake(INTAKE_SPEED);
            }
            else {
                intake.stop();
            }

            if (gamepad1.a) {
                THROTTLE = .4;
            }
            if (gamepad1.b) {
                THROTTLE = .75;
            }

            //gamepad 2

            if (gamepad2.dpad_up) {
                liftAtEncoderPos = false;
                arm.lift(SLIDE_SPEED);
                liftPosition = arm.getLiftPos();

            } else if (gamepad2.dpad_down) {
                liftAtEncoderPos = false;
                arm.lower(SLIDE_SPEED);
                liftPosition = arm.getLiftPos();


            } else {
                arm.liftToPosition(liftPosition, SLIDE_SPEED);
            }


            Log.d("RT_STICK", "" + gamepad2.right_stick_y);
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                pivotAtEncoderPos = false;
                arm.pivotForward(gamepad2.right_stick_y / 2);
                Log.d("IN_IF", "" + gamepad2.right_stick_y);
                pivotPosition = arm.getPivotPos();
            }

             else {
                 Log.d("SET_PIV", "" + pivotPosition + " " + gamepad2.right_stick_y);
                arm.pivotToPosition(pivotPosition, PIVOT_SPEED);
            }

            if (gamepad2.a) {
                pixelGrabber.onPress();

            } else {
                pixelGrabber.onRelease();
            }

            if (gamepad2.b || gamepad2.left_bumper) {
                pixelGrabber.onPressPivot();

            } else {
                pixelGrabber.onReleasePivot();
            }

            if (gamepad2.right_trigger > 0.05) {
                liftAtEncoderPos = true;
                arm.liftToPosition(SLIDE_POS_2, SLIDE_SPEED);
            }
            if (gamepad2.left_trigger > 0.05) {
                pivotAtEncoderPos = true;
                arm.pivotToPosition(PIVOT_POS_2, PIVOT_SPEED);
            }
            if (gamepad2.right_bumper) {
                liftAtEncoderPos = true;
                arm.liftToPosition(SLIDE_RESET, SLIDE_SPEED);
            }
//            if (gamepad2.left_bumper) {
//                arm.pivotToPosition(PIVOT_RESET, PIVOT_SPEED);
//            }

            if (gamepad2.x) {
                pixelGrabber.rotate(pixelGrabber.getWristPostion() + 0.01);
            }

            if (gamepad2.y) {
                pixelGrabber.rotate(pixelGrabber.getWristPostion() - 0.01);
            }


            telemetry.addData("wristServo pos: ", hardwareStore.wristServo.getPosition());

        }


        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");

        shutDown();
    }

    private void shutDown(){
        mecanumDrivetrain.stop(); 
    }
} */