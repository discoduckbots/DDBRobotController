package org.firstinspires.ftc.teamcode.discoduckbots.opmode.generate;

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
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Autonomous(name="RedLeft", group="Robot")
public class RedLeftAutonomous extends LinearOpMode{

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
        duckSensor = new DuckSensorTensorFlow(hardwareMap, false);
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
        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(0.2010497776976018  ,-5.450504644317084 ,6.273812958041638 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_1_0.end()).lineToLinearHeading( new Pose2d(28.417880045227395  ,5.000156793086839 ,1.6145305933334582 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1_1.end()).lineToLinearHeading( new Pose2d(29.017486775266118  ,-37.53386919150146 ,1.6175909522356484 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_1_2.end()).lineToLinearHeading( new Pose2d(29.09862928787415  ,-39.38868838305406 ,1.6139567760393057 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_4 = drive.trajectoryBuilder(trajectory_1_3.end()).lineToLinearHeading( new Pose2d(28.952786591363083  ,-35.828460079553125 ,1.621416400863378 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_5 = drive.trajectoryBuilder(trajectory_1_4.end()).lineToLinearHeading( new Pose2d(1.4509636591244899  ,-37.81163016697837 ,1.6986904631435316 ), velocityConstraint, accelerationConstraint).build();
        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        waitForStart();


        if (opModeIsActive()) {
            pixelMechanism.toGrab(this);
            drive.followTrajectory(trajectory_1_0 );
            drive.followTrajectory(trajectory_1_1 );
            pixelMechanism.openLeftGrabber();
            sleep(1000);
            pixelMechanism.toScore(this);
            drive.followTrajectory(trajectory_1_2 );
            arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_POWER);
            drive.followTrajectory(trajectory_1_3);
            pixelMechanism.openRightGrabber();
            sleep(1000);
            pixelMechanism.closeRightGrabber();
            drive.followTrajectory(trajectory_1_4);
            arm.liftToPosition(0, .5);
            pixelMechanism.toGrab(this);
            drive.followTrajectory(trajectory_1_5);
            pixelMechanism.pivotToPosition(0, .5);
            pixelMechanism.flipToPosition(0, .5);


        }
    }
}