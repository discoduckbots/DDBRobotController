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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private static double LIFT_POWER = .5;
    private static double EXTEND_POWER = .5;

    boolean liftAtEncoderPos = false;
    boolean pivotAtEncoderPos = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;
    private Intake intake = null;
    private Arm arm = null;
    //private DcMotor hangMotor = null;
    private PixelMechanism pixelMechanism = null;
    private FlipStateMachine flipStateMachine = null;
    boolean flipMotorAtEncoderPos = false;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();
        //hangMotor = hardwareStore.getHangMotor();
        flipStateMachine = hardwareStore.getFlipStateMachine();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean extendingManually = false;

        while (opModeIsActive()) {

            Log.d("LIFT" , "pos : " + hardwareStore.liftMotor.getCurrentPosition());

            Log.d("ARM", "lift pos : " + hardwareStore.liftMotor.getCurrentPosition() +
                    " extend pos : " + hardwareStore.extensionMotor.getCurrentPosition());

            Log.d("FLIP" , "pos : " + hardwareStore.flipMotor.getCurrentPosition());

             //Gamepad 1
            mecanumDrivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, THROTTLE);


            if (gamepad1.a) {
                THROTTLE = .4;
            }
            if (gamepad1.b) {
                THROTTLE = .75;
            }

            /*if (gamepad1.left_trigger > 0.05) {
                hangMotor.setDirection(DcMotor.Direction.FORWARD);
                hangMotor.setPower(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0.05) {
                hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                hangMotor.setPower(gamepad1.right_trigger);
            }
            else {
                hangMotor.setPower(0);
            } */

            //gamepad 2

            /*if (gamepad2.a) {
                pixelMechanism.increasePosition(pixelMechanism.leftHook, "leftHook");
            }
            if (gamepad2.b) {
                pixelMechanism.decreasePosition(pixelMechanism.leftHook, "leftHook");
            }
            if (gamepad2.x) {
                pixelMechanism.increasePosition(pixelMechanism.rightHook, "rightHook");
            }
            if (gamepad2.y) {
                pixelMechanism.decreasePosition(pixelMechanism.rightHook, "rightHook");
            } */

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

            /*if (gamepad2.a) {
                pixelMechanism.onPressLeftGrabber();
                pixelMechanism.onPressRightGrabber();
            } else {
                pixelMechanism.onReleaseLeftGrabber();
                pixelMechanism.onReleaseRightGrabber();
            }*/


            /*if (gamepad2.left_stick_y != 0) {
                Log.d("LEFT_STICK", "value " + )
                arm.extendForward(gamepad2.left_stick_x);
            }
            else {
                arm.stopExtend();
            }*/

            if (gamepad1.left_bumper) {
                pixelMechanism.grabFlipHook();

            }
            if (gamepad1.right_bumper) {
                pixelMechanism.resetIntake(this);
            }

            /*if(gamepad1.left_bumper) { //open
                pixelMechanism.intakeLeft(1);
                pixelMechanism.intakeRight(0);
            }
            if(gamepad1.right_bumper) { //close
                pixelMechanism.intakeLeft(0);
                pixelMechanism.intakeRight(1);
            } */
            if (gamepad2.right_bumper) {
                arm.onPressArm();
            } else {
                arm.onReleaseArm();
            }
            if (gamepad2.left_bumper) {
                arm.liftToPosition(0, LIFT_POWER);
            }

            if(gamepad2.dpad_down) {
                extendingManually = true;
                arm.extendBackward(EXTEND_POWER);
            }
            else if (gamepad2.dpad_up) {
                extendingManually = true;
                arm.extendForward(EXTEND_POWER);
            }
            else {
                if (extendingManually) {
                    arm.stopExtend();
                }
            }

            if(gamepad2.a) {
                extendingManually = false;
                arm.extendToPosition(Arm.EXTEND_IN, EXTEND_POWER);
            }

            if(gamepad2.b) {
                extendingManually = false;
                arm.extendToPosition(Arm.EXTEND_OUT, EXTEND_POWER);
            }

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