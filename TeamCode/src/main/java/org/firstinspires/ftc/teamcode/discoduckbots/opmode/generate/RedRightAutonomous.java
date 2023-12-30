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

    private static final double AUTONOMOUS_SPEED = 0.5;

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

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL * AUTONOMOUS_SPEED,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * AUTONOMOUS_SPEED);
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(28.431150655885965  ,-0.2459178243639267 ,6.271036508143791 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(22.917713875325276  ,-0.0817562078060555 ,6.255416623669204 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(26.646722116619927  ,-38.2446722856061 ,1.5700876468160505 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(4.621983362160702  ,-33.305430811165245 ,1.6194541952789496 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(28.907379980810653  ,3.147741594887895 ,1.6182971667993566 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_1_0.end()).lineToLinearHeading( new Pose2d(28.573443508450513  ,-38.04058030993238 ,1.5878287501699226 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1_1.end()).lineToLinearHeading( new Pose2d(28.589000167270125  ,-37.959045816080724 ,1.5903356452090591 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_1_2.end()).strafeLeft(24).build();
             //   lineToLinearHeading( new Pose2d(28.589000167270125  ,-37.959045816080724 ,1.5903356452090591 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(21.539860680873545  ,-11.448070861875262 ,0.08870551676927008 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(15.23173752765713  ,-12.02890547767913 ,0.07559252733381339 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(20.291540101891623  ,-37.028207843611845 ,1.5928425402481725 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(3.2661938032658817  ,-35.527535214674245 ,1.6069197200832992 ), velocityConstraint, accelerationConstraint).build();


        pixelMechanism.intakeLeft(pixelMechanism.LG_CLOSE_POS);
       // pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);
        waitForStart();

        if (opModeIsActive()) {
            int duckPos = duckSensor.getDuckPos();
            if (duckPos == DuckSensorTensorFlow.CLOSER) {
                pixelMechanism.flipToPosition(-124, 0.75);
                sleep(1000);
                drive.followTrajectory(trajectory_2_0 );
                sleep(1000);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_2_1 );
                sleep(350);
                //arm.extendToPosition(arm.EXTEND_OUT, 0.5);
                arm.liftToPosition(arm.LIFT_ROW2, 0.5);
                sleep(1000);
                drive.followTrajectory(trajectory_2_2 );
                //pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);
                //arm.extendToPosition(0, .5);
                sleep(1000);
                drive.followTrajectory(trajectory_2_3 );
            } else if (duckPos == DuckSensorTensorFlow.CENTER) {
                pixelMechanism.flipToPosition(-124, 0.75);
                sleep(1000);
                //drive forward
                drive.followTrajectory(trajectory_0);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_1);
                sleep(350);
               // arm.extendToPosition(arm.EXTEND_OUT, 0.5);
                arm.liftToPosition(arm.LIFT_ROW2, 0.5);
                //drive to board
                drive.followTrajectory(trajectory_2);
                sleep(350);
               // pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);
               // arm.extendToPosition(0, .5);
                //drive to side
                drive.followTrajectory(trajectory_3);

            } else { // far from board
                pixelMechanism.flipToPosition(-124, 0.75);
                sleep(1000);
                drive.followTrajectory(trajectory_1_0);
                sleep(350);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_1_1);
                sleep(350);
               // arm.extendToPosition(arm.EXTEND_OUT, 0.5);
                arm.liftToPosition(arm.LIFT_ROW2, 0.5);
                drive.followTrajectory(trajectory_1_2);
                sleep(350);
                //pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);
                drive.followTrajectory(trajectory_1_3);

            }
        }
    }
}
