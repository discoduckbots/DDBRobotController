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
@Autonomous(name="BlueBoard", group="Robot")
public class BlueBoardAuto extends LinearOpMode{

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

        Trajectory trajectory_4_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(22  ,11.25 ,0.027734502551036933 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_1 = drive.trajectoryBuilder(trajectory_4_0.end()).lineToLinearHeading( new Pose2d(7.851241670238913  ,9.003304983062003 ,0.05527773267069147 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_2 = drive.trajectoryBuilder(trajectory_4_1.end()).lineToLinearHeading( new Pose2d(14.41964524136867  ,34.74617993026693 ,4.716281549261342 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_3 = drive.trajectoryBuilder(trajectory_4_2.end()).lineToLinearHeading( new Pose2d(21.5  ,41.5 ,4.709395741731431 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_4 = drive.trajectoryBuilder(trajectory_4_3.end()).lineToLinearHeading( new Pose2d(16.25753424058732  ,32.96755182190088 ,4.705379020672311 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_5 = drive.trajectoryBuilder(trajectory_4_4.end()).lineToLinearHeading( new Pose2d(-0.5801152918707295  ,39.950172229511935 ,4.7007884823190444 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_5_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(30  ,0.8624158103143039 ,6.280890038002891 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_1 = drive.trajectoryBuilder(trajectory_5_0.end()).lineToLinearHeading( new Pose2d(13.362493646025605  ,7.300283948273949 ,0.032898858198427305 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_2 = drive.trajectoryBuilder(trajectory_5_1.end()).lineToLinearHeading( new Pose2d(22.85568377725055  ,38.42733106981765 ,4.735982609694069 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_3 = drive.trajectoryBuilder(trajectory_5_2.end()).lineToLinearHeading( new Pose2d(29.5  ,43 ,4.716664094124052 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_4 = drive.trajectoryBuilder(trajectory_5_3.end()).lineToLinearHeading( new Pose2d(22.561282613093322  ,34.2403741327491 ,4.711691010908002 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_5 = drive.trajectoryBuilder(trajectory_5_4.end()).lineToLinearHeading( new Pose2d(0.3955945738936819  ,33.70807611841769 ,4.706717927691958 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_6_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(0.1585136223815245  ,9.426054738107656 ,0.014919249648150945 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_1 = drive.trajectoryBuilder(trajectory_6_0.end()).lineToLinearHeading( new Pose2d(26.922743114517488  ,-3.1226945892925 ,4.741338237772939 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_2 = drive.trajectoryBuilder(trajectory_6_1.end()).lineToLinearHeading( new Pose2d(29.75179466529888  ,39.134612279607396 ,4.742103327498462 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_3 = drive.trajectoryBuilder(trajectory_6_2.end()).lineToLinearHeading( new Pose2d(30.137125386066973  ,42 ,4.69160740561243 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_4 = drive.trajectoryBuilder(trajectory_6_3.end()).lineToLinearHeading( new Pose2d(30.53057844341943  ,33.468763367017814 ,4.730626981615266 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_5 = drive.trajectoryBuilder(trajectory_6_4.end()).lineToLinearHeading( new Pose2d(1.3693139307776434  ,36.14322496380339 ,4.735026247537164 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        waitForStart();


        if (opModeIsActive()) {
            if (duckSensor.getDuckPos() == CLOSER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_4_0);
                sleep(500);
                pixelMechanism.openRightGrabber();
                sleep(WAIT_TIME);
                drive.followTrajectory(trajectory_4_1);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_4_2);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_4_3);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_4_4);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0, .5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_4_5);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if(duckSensor.getDuckPos() == CENTER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_5_0);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                sleep(WAIT_TIME);
                drive.followTrajectory(trajectory_5_1);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_5_2);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_5_3);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_5_4);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_5_5);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if (duckSensor.getDuckPos() == FARTHER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_6_0);
                drive.followTrajectory(trajectory_6_1);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                sleep(WAIT_TIME);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_6_2);
                arm.liftToPosition(arm.LIFT_AUTO_POS, arm.LIFT_2_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_6_3);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_6_4);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0, .5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_6_5);
                pixelMechanism.pivotToPosition(0, .5);
                pixelMechanism.flipToPosition(0, .5);
                pixelMechanism.toInit(this);
                sleep(1000);

            }

        }
    }
}