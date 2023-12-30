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

import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism.LG_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism.LG_OPEN_POS;
import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism.RG_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism.RG_OPEN_POS;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.FlipStateMachine;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;


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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="NEw Centerstage Opmode", group= "Linear Opmode")

public class NewMecanumDrivetrainTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.75;
    private static double INTAKE_SPEED = 0.75;
    private static double OUTAKE_SPEED = 1.0;
    private static double SLIDE_SPEED = 0.75;
    private static double PIVOT_SPEED = 0.5;
    private static double HANG_SPEED = 0.3;
    private static double LIFT_POWER = 1.0;
    private static double EXTEND_POWER = .5;
    private int liftPosition = 0;
    private int pivotPosition = 0;
    private int flipPosition = 0;
    boolean liftAtEncoderPos = false;
    boolean pivotAtEncoderPos = false;
    boolean flipAtEncoderPos = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;
    private Intake intake = null;
    private Arm arm = null;
    //private DcMotor hangMotor = null;
    private PixelMechanism pixelMechanism = null;
    private FlipStateMachine flipStateMachine = null;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();
        //hangMotor = hardwareStore.getHangMotor();
        flipStateMachine = hardwareStore.getFlipStateMachine();

        hardwareStore.flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareStore.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hardwareStore.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean extendingManually = false;

        while (opModeIsActive()) {

            Log.d("LIFT" , "pos : " + hardwareStore.liftMotor.getCurrentPosition());

            Log.d("FLIP" , "pos : " + hardwareStore.flipMotor.getCurrentPosition());

            Log.d("PIVOT" , "pos : " + hardwareStore.pivotMotor.getCurrentPosition());

             //Gamepad 1
            mecanumDrivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, THROTTLE);


            if (gamepad1.x) {
                THROTTLE = .4;
            }
            if (gamepad1.y) {
                THROTTLE = .75;
            }

            //gamepad 2

            if (gamepad2.left_bumper) {
                pixelMechanism.increasePosition(pixelMechanism.leftGrabber, "leftGrabber");
            }
            if (gamepad2.left_trigger > 0.01) {
                pixelMechanism.decreasePosition(pixelMechanism.leftGrabber, "leftGrabber");
            }
            if (gamepad2.right_bumper) {
                pixelMechanism.increasePosition(pixelMechanism.rightGrabber, "rightGrabber");
            }
            if (gamepad2.right_trigger > 0.01) {
                pixelMechanism.decreasePosition(pixelMechanism.rightGrabber, "rightGrabber");
            }

            /*if (gamepad2.dpad_up) {
                Log.d("dpadup " , "setpower 0.45");
                pixelMechanism.flipMotor.setPower(0.45);
            }
            else if (gamepad2.dpad_down) {
                pixelMechanism.flipMotor.setPower(-0.45);
                Log.d("dpadup " , "setpower - 0.45");
            }
            else {
                Log.d("dpadup " , "setpower 0");
                pixelMechanism.flipMotor.setPower(0);
            } */
            /*if (gamepad1.dpad_up) {
                arm.extendForward(.45);
            } else if (gamepad1.dpad_down) {
                arm.extendBackward(.45);
            } else {
                arm.stopExtend();
            } */

            if (gamepad1.left_bumper) {
                pixelMechanism.intakeLeft(LG_CLOSE_POS);
                pixelMechanism.intakeRight(RG_CLOSE_POS);
            }

            if (gamepad1.a) {
                pixelMechanism.intakeLeft(LG_CLOSE_POS);
            }

            if (gamepad1.b) {
                pixelMechanism.intakeRight(RG_CLOSE_POS);
            }

            if (gamepad1.x) {
                pixelMechanism.toScore();
            }

            if (gamepad2.a) {
                pixelMechanism.intakeLeft(LG_OPEN_POS);
                pixelMechanism.intakeRight(RG_OPEN_POS);
            }

            if (gamepad2.b) {
                pixelMechanism.intakeLeft(LG_OPEN_POS);
            }

            if (gamepad2.x) {
                pixelMechanism.intakeRight(RG_OPEN_POS);
            }

            /*if (gamepad2.y) {
                pixelMechanism.toGrab(this);
            } */

            /*if (gamepad2.left_stick_y != 0) {
                Log.d("LEFT_STICK", "value " + )
                arm.extendForward(gamepad2.left_stick_x);
            }
            else {
                arm.stopExtend();
            }*/

            /*if (gamepad1.left_bumper) {
                pixelMechanism.newGrabFlipHook(this);

            } */

            if(Math.abs(gamepad2.left_stick_y) > 0.01) {
                flipAtEncoderPos = false;
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelMechanism.flipMotor.setPower(gamepad2.left_stick_y);
                flipPosition = pixelMechanism.flipMotor.getCurrentPosition();
            } else {
                pixelMechanism.flipMotor.setTargetPosition(flipPosition);
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pixelMechanism.flipMotor.setPower(1.0);
            }

            if(gamepad2.y) {
                flipAtEncoderPos = true;
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pixelMechanism.flipMotor.setTargetPosition(832);
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pixelMechanism.flipMotor.setPower(1.0);
            }

            /*if(Math.abs(gamepad2.left_stick_y) > 0.01) {
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelMechanism.flipMotor.setPower(gamepad2.left_stick_y);
            } else if(gamepad2.y) {
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pixelMechanism.flipMotor.setTargetPosition(832);
            } else {
                pixelMechanism.flipMotor.setPower(0);
            } */

            if(gamepad2.dpad_left) {
                pixelMechanism.pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelMechanism.pivotMotor.setPower(PIVOT_SPEED);
                pivotPosition = pixelMechanism.pivotMotor.getCurrentPosition();
            } else if (gamepad2.dpad_right) {
                pixelMechanism.pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelMechanism.pivotMotor.setPower(-PIVOT_SPEED);
                pivotPosition = pixelMechanism.pivotMotor.getCurrentPosition();
            } else {
                pixelMechanism.pivotMotor.setTargetPosition(pivotPosition);
                pixelMechanism.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pixelMechanism.pivotMotor.setPower(PIVOT_SPEED);
            }

            if(gamepad2.dpad_up) {
                arm.lift(LIFT_POWER);
                liftPosition = arm.getLiftPos();
            } else if (gamepad2.dpad_down) {
                arm.lower(LIFT_POWER);
                liftPosition = arm.getLiftPos();
            } else {
                arm.liftMotor.setTargetPosition(liftPosition);
                arm.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.liftMotor.setPower(.5);
            }


           /* if (gamepad2.right_bumper) {
                arm.onPressArm();
            } else {
                arm.onReleaseArm();
            }
            if (gamepad2.left_bumper) {
                arm.liftToPosition(0, LIFT_POWER);
            }
            if(gamepad2.y) {
                pixelMechanism.intakeLeft(.5);
                pixelMechanism.intakeRight(.5);
            }

            if(gamepad2.dpad_down) {
                extendingManually = true;
                //arm.extendBackward(EXTEND_POWER);
            }
            else if (gamepad2.dpad_up) {
                extendingManually = true;
                //arm.extendForward(EXTEND_POWER);
            }
            else {
                if (extendingManually) {
                    //arm.stopExtend();
                }
            }

            if(gamepad2.a) {
                extendingManually = false;
                //arm.extendToPosition(arm.EXTEND_IN, EXTEND_POWER);
            }

            if(gamepad2.b) {
                extendingManually = false;
                //arm.extendToPosition(arm.EXTEND_OUT, EXTEND_POWER);
            } */

            /*if(gamepad2.x) { //close
                pixelMechanism.hookLeft(1);
                pixelMechanism.hookRight(0);
            }
            if(gamepad2.y) { //release
                pixelMechanism.hookLeft(0);
                pixelMechanism.hookRight(1);
            }*/




        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");

        shutDown();
    }

    private void shutDown(){
        mecanumDrivetrain.stop(); 
    }
}