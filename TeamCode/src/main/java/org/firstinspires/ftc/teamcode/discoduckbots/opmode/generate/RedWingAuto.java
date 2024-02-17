package org.firstinspires.ftc.teamcode.discoduckbots.opmode.generate;

import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.DuckSensorTensorFlow.CENTER;
import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.DuckSensorTensorFlow.CLOSER;
import static org.firstinspires.ftc.teamcode.discoduckbots.hardware.DuckSensorTensorFlow.FARTHER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.DuckSensorTensorFlow;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Autonomous(name="RedWing", group="Robot")
public class RedWingAuto extends LinearOpMode{

    private static final double STRAFE_SPEED = .5 ;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double AUTONOMOUS_SPEED = 0.5;

    private DuckSensorTensorFlow duckSensor =null;
    private Arm arm;
    private PixelMechanism pixelMechanism;
    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        duckSensor = new DuckSensorTensorFlow(hardwareMap, true);
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL * AUTONOMOUS_SPEED,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * AUTONOMOUS_SPEED);

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(25.203007812016107  ,5.94528395929207 ,0.5085933950567192 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_1_0.end()).lineToLinearHeading( new Pose2d(5.213992459671129  ,-0.12882823315687109 ,6.2732391407474815 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1_1.end()).lineToLinearHeading( new Pose2d(50.33106486946866  ,-0.17566735880394646 ,0.04341884192472367 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_1_2.end()).lineToLinearHeading( new Pose2d(49.36827145042456  ,1 ,1.5837357318802496 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_4 = drive.trajectoryBuilder(trajectory_1_3.end()).lineToLinearHeading( new Pose2d(49.45468576399666  ,-74.31878308666862 ,1.5083743939139715 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_5 = drive.trajectoryBuilder(trajectory_1_4.end()).lineToLinearHeading( new Pose2d(31.662297480498317  ,-84.87494656021451 ,1.546055062897123 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_6 = drive.trajectoryBuilder(trajectory_1_5.end()).lineToLinearHeading( new Pose2d(31  ,-88.5 ,1.6397785542765098 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_7 = drive.trajectoryBuilder(trajectory_1_6.end()).lineToLinearHeading( new Pose2d(32.588511981565325  ,-81.12189752279126 ,1.594255715606522 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_8 = drive.trajectoryBuilder(trajectory_1_7.end()).lineToLinearHeading( new Pose2d(49.870279819202885  ,-82.02888571069201 ,1.682049761612923 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(30  ,2 ,0.08358605251590223 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_2_0.end()).lineToLinearHeading( new Pose2d(2.3800415292188326  ,2 ,0.06522389910279536 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_2_1.end()).lineToLinearHeading( new Pose2d(15.267743263936035  ,16.015085529352806 ,6.279359858551846 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2_2.end()).lineToLinearHeading( new Pose2d(51.35935203836575  ,16.618337529878318 ,0.03787194141451611 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_4 = drive.trajectoryBuilder(trajectory_2_3.end()).lineToLinearHeading( new Pose2d(50.18083804033668  ,17.39361616731772 ,1.6501072655713749 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_5 = drive.trajectoryBuilder(trajectory_2_4.end()).lineToLinearHeading( new Pose2d(51.80017214860947  ,-70.65392303416228 ,1.5588703158000126 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_6 = drive.trajectoryBuilder(trajectory_2_5.end()).lineToLinearHeading( new Pose2d(25  ,-90 ,1.6260069392166816 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_7 = drive.trajectoryBuilder(trajectory_2_6.end()).lineToLinearHeading( new Pose2d(28.018476819186052  ,-79.68594969590839 ,1.6038193371758505 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_8 = drive.trajectoryBuilder(trajectory_2_7.end()).lineToLinearHeading( new Pose2d(48.02909308148708  ,-82.45332896055854 ,1.628876025687478 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_3_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(22.74018615833937  ,8.122559796332215 ,0.05757300184733616 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_1 = drive.trajectoryBuilder(trajectory_3_0.end()).lineToLinearHeading( new Pose2d(25.314098645159135  ,-2.848293008570664 ,4.875802757037675 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_2 = drive.trajectoryBuilder(trajectory_3_1.end()).lineToLinearHeading( new Pose2d(21.917932131407895  ,17.982665134501325 ,4.845199168015842 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_3 = drive.trajectoryBuilder(trajectory_3_2.end()).lineToLinearHeading( new Pose2d(50  ,19.033317930439626 ,0.08894168059473095 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_4 = drive.trajectoryBuilder(trajectory_3_3.end()).lineToLinearHeading( new Pose2d(50  ,20.158531752377232 ,1.5462463353285205 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_5 = drive.trajectoryBuilder(trajectory_3_4.end()).lineToLinearHeading( new Pose2d(53.32251746202466  ,-65.61063554335179 ,1.5626957644277653 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_6 = drive.trajectoryBuilder(trajectory_3_5.end()).lineToLinearHeading( new Pose2d(22.5  ,-84.64812741737151 ,1.5684339373693685 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_7 = drive.trajectoryBuilder(trajectory_3_6.end()).lineToLinearHeading( new Pose2d(20.360022211725752  ,-90 ,1.5856484561941508 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_8 = drive.trajectoryBuilder(trajectory_3_7.end()).lineToLinearHeading( new Pose2d(20.90330984408877  ,-80.96889617351157 ,1.6076447858035987 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_9 = drive.trajectoryBuilder(trajectory_3_8.end()).lineToLinearHeading( new Pose2d(49.304059295771054  ,-78.81611693366193 ,1.6384396472568241 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        waitForStart();


        if (opModeIsActive()) {
            if (duckSensor.getDuckPos() == FARTHER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_1_0);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_1_1);
                drive.followTrajectory(trajectory_1_2);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_1_3);
                drive.followTrajectory(trajectory_1_4);
                drive.followTrajectory(trajectory_1_5);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_1_6);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_1_7);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_1_8);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if(duckSensor.getDuckPos() == CENTER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_2_0);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_2_1);
                drive.followTrajectory(trajectory_2_2);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_2_3);
                drive.followTrajectory(trajectory_2_4);
                drive.followTrajectory(trajectory_2_5);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_2_6);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_2_7);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,0.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_2_8);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if (duckSensor.getDuckPos() == CLOSER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_3_0);
                drive.followTrajectory(trajectory_3_1);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_3_2);
                drive.followTrajectory(trajectory_3_3);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_3_4);
                drive.followTrajectory(trajectory_3_5);
                drive.followTrajectory(trajectory_3_6);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_3_7);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_3_8);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_3_9);
                pixelMechanism.toInit(this);
                sleep(1000);

            }

        }
    }
}