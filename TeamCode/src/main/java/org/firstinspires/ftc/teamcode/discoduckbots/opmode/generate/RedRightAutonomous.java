package org.firstinspires.ftc.teamcode.discoduckbots.opmode.generate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name="RedRight", group="Robot")
public class RedRightAutonomous extends LinearOpMode{

    private static final double STRAFE_SPEED = .5 ;
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;

    private static final double AUTONOMOUS_SPEED = 1;

    private static final double ROTATION_SPEED = 0.4;
    private static final int WOBBLE_GRABBER_REVOLUTIONS = 6250;
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

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL / AUTONOMOUS_SPEED,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / AUTONOMOUS_SPEED);
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading(new Pose2d(28.84780544717903, -0.318100765562541, 6.2494386431912865), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading(new Pose2d(16.41577103536217, 0.17534522251196422, 6.216656169602654), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading(new Pose2d(30.697340616359405, -40.02050037959612, 1.5395613789030227), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading(new Pose2d(30.69930741332339, -40.04860110181801, 1.5412969216224095), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(28.907379980810653  ,3.147741594887895 ,1.6182971667993566 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_1_0.end()).lineToLinearHeading( new Pose2d(28.573443508450513  ,-38.04058030993238 ,1.5878287501699226 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1_1.end()).lineToLinearHeading( new Pose2d(28.589000167270125  ,-37.959045816080724 ,1.5903356452090591 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.intakeLeft();
        waitForStart();

        if (opModeIsActive()) {
            int duckPos = duckSensor.getDuckPos();
            if (duckPos == DuckSensorTensorFlow.CLOSER) {

            } else if (duckPos == DuckSensorTensorFlow.CENTER) {
                pixelMechanism.flipToPosition(-104, 0.5);
                drive.followTrajectory(trajectory_0);
                pixelMechanism.intakeLeft(1);
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_1);
                drive.followTrajectory(trajectory_2);
                drive.followTrajectory(trajectory_3);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                pixelMechanism.outtakeRight();

            } else {
                pixelMechanism.flipToPosition(-104, 0.5);
                drive.followTrajectory(trajectory_1_0);
                pixelMechanism.outtakeLeft();
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_1_1);
                drive.followTrajectory(trajectory_1_2);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                pixelMechanism.outtakeRight();
            }
        }
    }
}
