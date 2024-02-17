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
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

//@Disabled
@Autonomous(name="RedBoard", group="Robot")
public class RedBoardAuto extends LinearOpMode{

    private static final double STRAFE_SPEED = .5 ;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double AUTONOMOUS_SPEED = 0.5;
    private static int WAIT_TIME = 0;

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

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(3  ,-7.600532811765655 ,6.281655127728495 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_1_0.end()).lineToLinearHeading( new Pose2d(28.20806544666193  ,4.628715782240674 ,1.5424208867007714 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1_1.end()).lineToLinearHeading( new Pose2d(29.372431404010033  ,-35.95124173525746 ,1.6317451121582707 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_1_2.end()).lineToLinearHeading( new Pose2d(26.5  ,-41.62217368942844 ,1.6164433176473505 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_4 = drive.trajectoryBuilder(trajectory_1_3.end()).lineToLinearHeading( new Pose2d(30.049083674072808  ,-35.51164777854881 ,1.566329940624093 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_5 = drive.trajectoryBuilder(trajectory_1_4.end()).lineToLinearHeading( new Pose2d(1.036570519181165  ,-35.4416084454411 ,1.575511017330646 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(30.055991728312314  ,-1.2675202559266763 ,6.267692240237263 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_2_0.end()).lineToLinearHeading( new Pose2d(23.515287061907316  ,-1.186149259965567 ,6.270943871570836 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_2_1.end()).lineToLinearHeading( new Pose2d(20.85052108327826  ,-35.6400003904783 ,1.5665984216197302 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2_2.end()).lineToLinearHeading( new Pose2d(27  ,-41.829688488432545 ,1.5719540496985376 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_4 = drive.trajectoryBuilder(trajectory_2_3.end()).lineToLinearHeading( new Pose2d(24.35902136843372  ,-33.90387493054701 ,1.562581700560595 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_5 = drive.trajectoryBuilder(trajectory_2_4.end()).lineToLinearHeading( new Pose2d(1.039960287485851  ,-37.31381852649121 ,1.6155641640546516 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_3_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(22  ,-14 ,0.042271207336409766 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_1 = drive.trajectoryBuilder(trajectory_3_0.end()).lineToLinearHeading( new Pose2d(9.0635737400733  ,-13.569947682889088 ,0.052599918631281106 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_2 = drive.trajectoryBuilder(trajectory_3_1.end()).lineToLinearHeading( new Pose2d(18.704528131335884  ,-34.13812830291473 ,1.6195036765495239 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_3 = drive.trajectoryBuilder(trajectory_3_2.end()).lineToLinearHeading( new Pose2d(20.550611840828342  ,-40.91831542759491 ,1.6087924203918842 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_4 = drive.trajectoryBuilder(trajectory_3_3.end()).lineToLinearHeading( new Pose2d(20.43645358809851  ,-35.543940469365445 ,1.596550984783149 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3_5 = drive.trajectoryBuilder(trajectory_3_4.end()).lineToLinearHeading( new Pose2d(0.7952916538680732  ,-35.56883829046704 ,1.6009502507050373 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        waitForStart();


        if (opModeIsActive()) {
            if (duckSensor.getDuckPos() == FARTHER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_1_0);
                drive.followTrajectory(trajectory_1_1);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                sleep(WAIT_TIME);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_1_2);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_1_3);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_1_4);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_1_5);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if(duckSensor.getDuckPos() == CENTER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_2_0);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                sleep(WAIT_TIME);
                drive.followTrajectory(trajectory_2_1);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_2_2);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_2_3);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_2_4);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_2_5);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if (duckSensor.getDuckPos() == CLOSER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_3_0);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                sleep(WAIT_TIME);
                drive.followTrajectory(trajectory_3_1);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_3_2);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_3_3);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_3_4);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_3_5);
                pixelMechanism.toInit(this);
                sleep(1000);

            }

        }
    }
}