package org.firstinspires.ftc.teamcode.discoduckbots.opmode.generate;

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



import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism.FLIP_POWER;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;


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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Gen Autonomous Teleop", group="Linear Opmode")
public class NewAutonomousGenerator extends LinearOpMode {

    private SampleMecanumDrive sampleMecanumDrive = null;

    private Intake intake = null;
   // private Arm arm = null;
   // private PixelGrabber pixelGrabber = null;
    private static double THROTTLE = 0.5;
    private static double STRAFE_THROTTLE = 0.5;
    private static double TURN_THROTTLE = 0.7;
    private static double intakeSpeed = .81;
    private static final double ARM_SPEED = 1;
    private static final double LIFT_POWER = 0.75;
    private static final double LOWER_POWER = 0.25;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * THROTTLE,
            DriveConstants.MAX_ANG_VEL * THROTTLE,
            DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accelerationConstraint =
            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * THROTTLE);
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Arm arm;
    private PixelMechanism pixelMechanism;
    private int liftPosition = 0;


    @Override
    public void runOpMode() {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();
      //  arm = hardwareStore.getArm();
      //  pixelGrabber = hardwareStore.getPixelGrabber();

        //DistanceSensor distanceSensor = hardwareStore.getDistanceSensor();



        boolean coneArmAtEncoderPos = false;
        boolean coneTurretEncoderPos = false;

       /* Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
*/
       /* blockDetector = new BlockDetector(hardwareStore.getWebcamName(), hardwareMap, new BlockDetectorListener() {
            @Override
            public void onBlockDetected(boolean grabber, boolean zone1, boolean zone2) {
                Log.d("ftc-opencv", "Cargo grabber " + grabber + " zone1 " + zone1 + " zone2  " + zone2);
                if ((grabber && zone1) ||
                        (grabber && zone2) ||
                        (zone1 && zone2)) {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                } else if (grabber) {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                } else if (zone1){
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                } else if (zone2) {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                } else {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
            }
        }, hardwareStore.getBlockSensor());
*/
        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean gamepad1xRelease = true;
        boolean gamepad1yRelease = true;
        boolean gamepad1DpadLeftRelease = true;
        boolean gamepad1DpadRightRelease = true;
        boolean gamepad1LeftTriggerRelease = true;
        boolean gamepad1RightTriggerRelease = true;
        while (opModeIsActive()) {
            arm.print();
            pixelMechanism.print();
            Log.d("Tries", "" + tries);
            //pixelMechanism.intakeLeft(1);

            if (gamepad1.x && gamepad1xRelease) {
                pixelMechanism.toScore(this);
                gamepad1xRelease = false;
            } else if (!gamepad1.x) {
                gamepad1xRelease = true;
            }

            if (gamepad1.y && gamepad1yRelease) {
                pixelMechanism.toGrab(this);
                gamepad1yRelease = false;
            } else if (!gamepad1.y) {
                gamepad1yRelease = true;
            }

            if (gamepad1.a) {
                pixelMechanism.flipToPosition(0, FLIP_POWER);
            }

         //   Log.d("Pivot " , "pos : " + arm.getPivotCurrentPosition());
         //   Log.d("Motor " , "pos : " + arm.getLiftMotorPosition());

            if(gamepad1.dpad_up) {
                telemetry.addData("dpad up", "pressed");
                arm.lower(LIFT_POWER);
                liftPosition = arm.getLiftPos();
            } else if (gamepad1.dpad_down) {
                telemetry.addData("dpad down", "pressed");
                arm.lift(LOWER_POWER);
                liftPosition = arm.getLiftPos();
            } else {
                arm.liftMotor.setTargetPosition(liftPosition);
                arm.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.liftMotor.setPower(.5);
            }

            if (gamepad1.dpad_left && gamepad1DpadLeftRelease) {
                pixelMechanism.closeGrabbers();
                gamepad1DpadLeftRelease = false;
            } else {
                gamepad1DpadLeftRelease = true;
            }

            if (gamepad1.dpad_right && gamepad1DpadRightRelease) {
                pixelMechanism.openLeftGrabber();
                pixelMechanism.openRightGrabber();
                gamepad1DpadRightRelease = false;
            } else {
                gamepad1DpadRightRelease = true;
            }

            if (gamepad1.left_trigger > 0 && gamepad1LeftTriggerRelease) {
                tries++;
                gamepad1LeftTriggerRelease = false;
            } else {
                gamepad1LeftTriggerRelease = true;
            }

            if (gamepad1.right_trigger > 0 && gamepad1RightTriggerRelease) {
                tries = 0;
                gamepad1RightTriggerRelease = false;
            } else {
                gamepad1RightTriggerRelease = true;
            }

            /* Gamepad 1 */
            {
                sampleMecanumDrive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * THROTTLE,
                                -gamepad1.left_stick_x * STRAFE_THROTTLE,
                                -gamepad1.right_stick_x * TURN_THROTTLE
                        )
                );

                sampleMecanumDrive.update();

                Pose2d poseEstimate = sampleMecanumDrive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("Tries", tries);
                telemetry.update();
                Log.d("LOC", "x = " + poseEstimate.getX() +
                        " y= " + poseEstimate.getY() +
                        " heading " + Math.toDegrees(poseEstimate.getHeading()));
            }


            if (gamepad1.left_bumper) {
                Log.d("GEN", "LB " + gamepad1.left_bumper + " LBP " + leftBumperPressed);
                if(!leftBumperPressed){
                    leftBumperPressed = true;
                    addAutonomousPoint(sampleMecanumDrive);
                }
            } else{
                leftBumperPressed = false;
            }

            if (gamepad1.right_bumper) {
                if(!rightBumperPressed) {
                    rightBumperPressed = true;
                    completeAutonomousPath(sampleMecanumDrive);
                }
            }
            else{
                rightBumperPressed = false;
            }

        }

        telemetry.addData("MecanumOdometryTeleOp", "Stopping");

        shutDown();
    }
    public ArrayList<Trajectory> arrayList = new ArrayList<Trajectory>();
    private int tries = 1;

    private void completeAutonomousPath(SampleMecanumDrive drive) {
        Pose2d newEnd= new Pose2d (0,0,0);
        Trajectory firstTrajectory = drive.trajectoryBuilder(arrayList.get(arrayList.size() - 1).end())
                .lineToLinearHeading( newEnd,
                        velocityConstraint, accelerationConstraint)
                .build();

        Log.d("GEN", "Moving to " + firstTrajectory);
        drive.followTrajectory(firstTrajectory);
        for(Trajectory trajectory: arrayList){
            Log.d("GEN", "Moving to " + trajectory);
            drive.followTrajectory(trajectory);
        }
        try {
            printStatement(arrayList);

           // tries++;
            arrayList.clear();
            
        } catch (IOException e) {
            e.printStackTrace();
            Log.d("AUT", ":exception writing");
        }

    }

    public void printStatement(ArrayList<Trajectory> arrayList) throws IOException {
        File path =Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(path, "GeneratedAutonomous_"+ tries + ".java");
        Log.d("AUT", "filepath " + file.getAbsolutePath());
        FileWriter writer = new FileWriter(file);
        try {
            String AUTONOMOUS = "package org.firstinspires.ftc.teamcode.discoduckbots.opmode.generate;\n" +
                    "\n" +
                    "import com.acmerobotics.roadrunner.geometry.Pose2d;\n" +
                    "import com.acmerobotics.roadrunner.trajectory.Trajectory;\n" +
                    "import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;\n" +
                    "import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                    "import com.qualcomm.robotcore.util.ElapsedTime;\n" +
                    "\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.DuckSensorTensorFlow;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;\n" +
                    "import org.firstinspires.ftc.teamcode.drive.DriveConstants;\n" +
                    "import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;\n" +
                    "\n" +
                    "//@Disabled\n" +
                    "@Autonomous(name=\"BlueRight\", group=\"Robot\")\n" +
                    "public class BlueRightAutonomous extends LinearOpMode{\n" +
                    "\n" +
                    "    private static final double STRAFE_SPEED = .5 ;\n" +
                    "    private ElapsedTime runtime = new ElapsedTime();\n" +
                    "    \n" +
                    "    private static final double AUTONOMOUS_SPEED = 0.5;\n" +
                    "\n" +
                    "    private DuckSensorTensorFlow duckSensor =null;\n" +
                    "    private Arm arm;\n" +
                    "    private PixelMechanism pixelMechanism;" +
                    "\n" +
                    "    @Override\n" +
                    "    public void runOpMode() {\n" +
                    "        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);\n" +
                    "        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);\n" +
                    "        duckSensor = new DuckSensorTensorFlow(hardwareMap, false);\n" +
                    "        arm = hardwareStore.getArm();\n" +
                    "        pixelMechanism = hardwareStore.getPixelMechanism();\n" +
                    "\n" +
                    "        /** Wait for the game to begin */\n" +
                    "        telemetry.addData(\">\", \"Press Play to start op mode\");\n" +
                    "        telemetry.update();\n" +
                    "\n" +
                    "        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * AUTONOMOUS_SPEED,\n" +
                    "                DriveConstants.MAX_ANG_VEL * AUTONOMOUS_SPEED,\n" +
                    "                DriveConstants.TRACK_WIDTH);\n" +
                    "        TrajectoryAccelerationConstraint accelerationConstraint =\n" +
                    "                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * AUTONOMOUS_SPEED);\n";

            String AUTONOMOUS_PART2 = "        waitForStart();\n" +
                    "\n" +
                    "        if (opModeIsActive()) {\n";
            String AUTONOMOUS_PART3 = "       }\n" +
                    "    }\n" +
                    "}\n";
            writer.append(AUTONOMOUS);



            String TRAJ_CODE = "        Trajectory trajectory_TRIES_INDEX = drive.trajectoryBuilder(START)" +
                    ".lineToLinearHeading( new Pose2d(XPOS,YPOS,ZPOS), " +
                    "velocityConstraint, accelerationConstraint)" +
                    ".build();\n";

            StringBuilder code = new StringBuilder();

            for (int i = 0; i < arrayList.size(); i++) {
                String newValue = " ";
                String startReplacement;
                if (i == 0) {
                    startReplacement = "drive.getPoseEstimate()";
                } else {
                    startReplacement = "trajectory_TRIES_INDEX.end()".replace("INDEX", (i - 1) + "");
                    startReplacement = startReplacement.replace("TRIES", tries +"");

                }
                newValue = TRAJ_CODE.replace("INDEX", i + "");
                newValue = newValue.replace("START", startReplacement);
                newValue = newValue.replace("XPOS", arrayList.get(i).end().getX() + "  ");
                newValue = newValue.replace("YPOS", arrayList.get(i).end().getY() + " ");
                newValue = newValue.replace("ZPOS", arrayList.get(i).end().getHeading() + " ");
                newValue = newValue.replace("TRIES", tries + "");
                writer.append(newValue);
            }

            writer.append("        pixelMechanism.closeLeftGrabber();\n" +
                    "        pixelMechanism.closeRightGrabber();\n");
            writer.append(AUTONOMOUS_PART2);
            String FOLLOW = "        drive.followTrajectory(trajectory_TRIES_INDEX);\n";
            for (int i = 0; i < arrayList.size(); i++) {

                String newerValue = " ";
                newerValue = FOLLOW.replace("INDEX", i + " ");
                newerValue = newerValue.replace("TRIES", tries + "");
                writer.append(newerValue);
            }

            writer.append(AUTONOMOUS_PART3);
        } catch (Exception e) {
            Log.d("AUT", ":exception writing");
        }
        finally {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                Log.d("AUT", ":exception writing");
                e.printStackTrace();
            }
        }
    }
    private void addAutonomousPoint( SampleMecanumDrive drive  ) {

        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d end = new Pose2d(poseEstimate.getX(), poseEstimate.getY(),  poseEstimate.getHeading());
        Pose2d start;
        if(arrayList.size()==0){
            start = new Pose2d(0,0,0);
        }
        else{
            start = arrayList.get(arrayList.size() - 1).end();
        }
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading( end,
                        velocityConstraint, accelerationConstraint)
                .build();

        arrayList.add(trajectory);
        Log.d("GEN", "Adding Pos " + start + " to " + arrayList.size());
    }

    private void shutDown(){

    }

}
