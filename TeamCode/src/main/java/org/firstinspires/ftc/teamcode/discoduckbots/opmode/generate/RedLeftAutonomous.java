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
        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(30.542710554317654  ,-2.764686613778897 ,6.226377395057771 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_1_0.end()).lineToLinearHeading( new Pose2d(18.394201013620666  ,-2.3755652520048045 ,6.2361322890584745 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1_1.end()).lineToLinearHeading( new Pose2d(24.574243185525738  ,-42.55822590984887 ,1.5750144086007438 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_1_2.end()).forward(3, velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_4 = drive.trajectoryBuilder(trajectory_1_3.end()).strafeLeft(20,velocityConstraint, accelerationConstraint).build();
        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        waitForStart();


        if (opModeIsActive()) {
            pixelMechanism.toGrab(this);
            drive.followTrajectory(trajectory_1_0 );
            pixelMechanism.openLeftGrabber();
            sleep(1000);
            pixelMechanism.toScore(this);
            drive.followTrajectory(trajectory_1_1 );
            arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_POWER);
            drive.followTrajectory(trajectory_1_2 );
            pixelMechanism.openRightGrabber();
            sleep(1000);
            pixelMechanism.closeRightGrabber();

            drive.followTrajectory(trajectory_1_3);
            arm.liftToPosition(0, .5);
            pixelMechanism.toGrab(this);
            drive.followTrajectory(trajectory_1_4);
            pixelMechanism.pivotToPosition(0, .5);
            pixelMechanism.flipToPosition(0, .5);


        }
    }
}