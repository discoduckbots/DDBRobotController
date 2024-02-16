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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test Op Mode", group= "Linear Opmode")
@Disabled
public class TestTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.75;
    private static final double LIFT_POWER = 0.75;
    private static final double LOWER_POWER = 0.25;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Intake intake = null;
    private Arm arm = null;
    private PixelMechanism pixelMechanism = null;
    private MecanumDrivetrain mecanumDrivetrain = null;

    private int liftPosition = 0;
    boolean inGrabPosition = false;
    boolean inScorePosition = false;
    private int pivotPosition = 0;
    private int flipPosition = 0;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        waitForStart();

        /* Put in Grab Position */
        inGrabPosition = true;
        //pixelMechanism.toGrab(this);

        while (opModeIsActive()) {

            mecanumDrivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, THROTTLE);

           if(gamepad2.dpad_up) {
                telemetry.addData("dpad up", "pressed");
                pixelMechanism.liftPivot(1.0);
                pivotPosition = pixelMechanism.getPivotPos();

            } else if (gamepad2.dpad_down) {
                telemetry.addData("dpad down", "pressed");
                pixelMechanism.lowerPivot(LOWER_POWER);
                pivotPosition = pixelMechanism.getPivotPos();

            } else {
                pixelMechanism.pivotToPosition(pivotPosition, 0.75);
                //pixelMechanism.pivotMotor.setPower(0);
            }

            /*if(gamepad2.dpad_right) {
                telemetry.addData("dpad up", "pressed");
                pixelMechanism.liftFlip(LIFT_POWER);
                flipPosition = pixelMechanism.getFlipPos();

            } else if (gamepad2.dpad_left) {
                telemetry.addData("dpad down", "pressed");
                pixelMechanism.lowerFlip(LOWER_POWER);
                flipPosition = pixelMechanism.getFlipPos();

            } else {
                pixelMechanism.flipToPosition(flipPosition, 0.5);
                //pixelMechanism.flipMotor.setPower(0);
            } */

            if (gamepad2.dpad_right) {
                telemetry.addData("dpad up", "pressed");
                pixelMechanism.liftFlip(LIFT_POWER);
            }
            else if (gamepad2.dpad_left) {
                telemetry.addData("dpad down", "pressed");
                pixelMechanism.lowerFlip(LOWER_POWER);

            } else {
                pixelMechanism.flipMotor.setPower(0.0);
            }

            /*
            if (gamepad1.a) {
               frontLeft.setPower(0.5);
            }
            else {
                frontLeft.setPower(0);
            }

            if (gamepad1.b) {
                frontBack.setPower(0.5);
            }
            else {
                frontBack.setPower(0);
            }

            if (gamepad1.x) {
                backLeft.setPower(0.5);
            }
            else {
                backLeft.setPower(0);
            } */



            if (gamepad2.a) {
                pixelMechanism.toStack();
            }

            if (gamepad2.x) {
                pixelMechanism.openLeftGrabber();
            }
            if (gamepad2.y) {
                pixelMechanism.closeLeftGrabber();
            }

           /* if (gamepad2.dpad_up) {
                arm.lift(0.5);
            }
            else if (gamepad2.dpad_down) {
                arm.lower(0.5);
            }
            else {
                arm.stopLift();
            } */

            telemetry.addData("Pivot Position", pixelMechanism.pivotMotor.getCurrentPosition());
            telemetry.addData("Flip Position", pixelMechanism.flipMotor.getCurrentPosition());
            telemetry.addData("Lift Position", arm.liftMotor1.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");
    }

}