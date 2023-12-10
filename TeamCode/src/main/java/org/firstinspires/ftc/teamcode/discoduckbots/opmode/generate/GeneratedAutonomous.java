package org.firstinspires.ftc.teamcode.discoduckbots.opmode.generate;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Autonomous(name="Generated", group="Robot")
public class GeneratedAutonomous extends LinearOpMode{

    private static final double STRAFE_SPEED = .5 ;
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;

    private static final double AUTONOMOUS_SPEED = 1;

    private static final double ROTATION_SPEED = 0.4;
    private static final int WOBBLE_GRABBER_REVOLUTIONS = 6250;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL/AUTONOMOUS_SPEED,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint =
        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/AUTONOMOUS_SPEED);
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(33.686284274226125  ,1.1041419872037301 ,0.03181828318896418 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(18.462442205144974  ,0.27833799073156706 ,4.620535381995583 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(19.100284937737502  ,1.8521106471621476 ,1.5274125798671756 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(29.179120664232556  ,-34.29128340299693 ,1.5083216099537857 ), velocityConstraint, accelerationConstraint).build();
        waitForStart();

        if (opModeIsActive()) {
        drive.followTrajectory(trajectory_0 );
        drive.followTrajectory(trajectory_1 );
        drive.followTrajectory(trajectory_2 );
        drive.followTrajectory(trajectory_3 );
       }
    }
}
