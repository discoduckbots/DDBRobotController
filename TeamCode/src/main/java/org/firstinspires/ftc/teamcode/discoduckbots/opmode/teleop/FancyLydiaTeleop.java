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

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.DroneLauncher;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelDetector;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Fancy Op Mode", group= "Linear Opmode")
@Disabled
public class FancyLydiaTeleop extends LinearOpMode {

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    private static double THROTTLE = 0.75;
    private static final double LIFT_POWER = 0.75;
    private static double LOWER_POWER = 0.25;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Intake intake = null;
    private Arm arm = null;
    private PixelMechanism pixelMechanism = null;
    private DroneLauncher droneLauncher = null;
    private MecanumDrivetrain mecanumDrivetrain = null;
    private PixelDetector pixelDetector = null;
    private RevBlinkinLedDriver lights = null;

    private int liftPosition = 0;
    private int liftPosition2 = 0;
    boolean inGrabPosition = false;
    boolean inScorePosition = false;
    boolean isBackPressed = false;

    @Override

    public void runOpMode() {
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * THROTTLE,
                DriveConstants.MAX_ANG_VEL * THROTTLE,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * THROTTLE);
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();
        droneLauncher = hardwareStore.getDroneLauncher();
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        pixelDetector = hardwareStore.getPixelDetector();
        lights = hardwareStore.getLedDriver();
        waitForStart();

        /* Put in Grab Position */
        inGrabPosition = true;
        pixelMechanism.toGrab(this);

        while (opModeIsActive()) {

            mecanumDrivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, THROTTLE);

            //mecanumDrivetrain.drive(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_left, gamepad1.dpad_right, SLOW_THROTTLE);

            if(gamepad2.dpad_up) {
                telemetry.addData("dpad up", "pressed");
                arm.lower(LIFT_POWER);
                liftPosition = arm.getLiftPos();
                liftPosition2 = arm.getLiftPos2();
            } else if (gamepad2.dpad_down) {
                telemetry.addData("dpad down", "pressed");
                arm.lift(LOWER_POWER);
                liftPosition = arm.getLiftPos();
                liftPosition2 = arm.getLiftPos2();
            } else {
                arm.liftMotor1.setTargetPosition(liftPosition);
                arm.liftMotor2.setTargetPosition(liftPosition2);
                arm.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.liftMotor1.setPower(0.5);
                arm.liftMotor2.setPower(0.5);
            }

            if(gamepad1.left_bumper){
                THROTTLE = .75;
            }

            if (gamepad1.right_bumper){
                THROTTLE = .3;
            }



            if (gamepad1.y) {
                droneLauncher.launch();
            }

            if(gamepad2.left_bumper){
                pixelMechanism.openLeftGrabber();
                pixelMechanism.openRightGrabber();
            }
            if (gamepad2.left_trigger > 0.5){
                pixelMechanism.openLeftGrabber();
            }

            if (gamepad2.right_bumper){
                pixelMechanism.closeRightGrabber();
                pixelMechanism.closeLeftGrabber();
            }
            if (gamepad2.right_trigger > 0.5){
                pixelMechanism.openRightGrabber();
            }

            if (gamepad2.y){
                if (!inScorePosition){
                    inScorePosition = true;
                    inGrabPosition = false;
                    pixelMechanism.toScore(this);
                }
            }


            if(gamepad2.x) {
                if (!inGrabPosition){
                    inGrabPosition = true;
                    inScorePosition = false;
                    pixelMechanism.toGrab(this);
                }
            }

            if(gamepad2.y) {
                if (!inScorePosition){
                    inScorePosition = true;
                    inGrabPosition = false;
                    pixelMechanism.toScore(this);
                }
            }

            if (gamepad2.a) {
                LOWER_POWER = 0.85;
            }

            if (gamepad2.back) {
                isBackPressed = true;
                pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelMechanism.flipMotor.setPower(-0.5);
            }
            else {
                if (isBackPressed == true) {
                    pixelMechanism.flipMotor.setPower(0);
                    pixelMechanism.flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelMechanism.flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                isBackPressed = false;
            }



//            if (pixelMechanism.isPivotTouchSensorPressed()){
//                telemetry.addData("Pivot Touch Sensor", "pressed");
//                pixelMechanism.resetPivotEncoder();
//            }

            telemetry.addData("Pivot Position", pixelMechanism.pivotMotor.getCurrentPosition());
            telemetry.addData("Flip Position", pixelMechanism.flipMotor.getCurrentPosition());
            telemetry.addData("Lift Position", arm.liftMotor1.getCurrentPosition());
            telemetry.addData("Lift Position", arm.liftMotor2.getCurrentPosition());
            telemetry.addData("Drone Servo", droneLauncher.droneServo.getPosition());
            telemetry.update();
        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");
    }

}