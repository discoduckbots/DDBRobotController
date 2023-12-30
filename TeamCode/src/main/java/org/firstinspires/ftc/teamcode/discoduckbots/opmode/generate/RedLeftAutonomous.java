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

        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(22.067561198990784  ,-0.025956250469601375 ,0.022947731512052982 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(25.008092126757244  ,-4.641995333186972 ,5.340978448626011 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(7.08112010532648  ,12.18211128313812 ,0.9279368406385107 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(4.3941117163532235  ,-51.624400666117545 ,1.5735587322548295 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4 = drive.trajectoryBuilder(trajectory_3.end()).lineToLinearHeading( new Pose2d(22.91484051286949  ,-84.915102703212 ,1.6059555296835786 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(29.300902986998118  ,1.2532034978367506 ,0.02873287391004542 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(4.237831838899467  ,9.495063150152824 ,1.587828750169912 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(4.047294548075504  ,-49.71136412928731 ,1.5486826199434756 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(29.90381760963904  ,-85.26416288808537 ,1.6098122912823065 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(23.628367632661362  ,4.749119303030841 ,0.6998093920775394 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(2.989977755305425  ,7.576600611549671 ,1.5918783498484812 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(5.923393881274236  ,-45.61696378976919 ,1.6059555296836052 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(38.150964111315794  ,-84.75767127196804 ,1.6042199869641944 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.intakeLeft(pixelMechanism.LG_CLOSE_POS);
        //pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);
        waitForStart();

        if (opModeIsActive()) {
            int duckPos = duckSensor.getDuckPos();
            if (duckPos == DuckSensorTensorFlow.CLOSER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_0);
                sleep(350);
                drive.followTrajectory(trajectory_1);
                sleep(350);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(1000);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_2);
                sleep(350);
                //arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                drive.followTrajectory(trajectory_3 );
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                sleep(350);
                drive.followTrajectory(trajectory_4 );
                sleep(2000);
               // pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);

            } else if (duckPos == DuckSensorTensorFlow.CENTER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                //drive forward
                drive.followTrajectory(trajectory_1_0);
                sleep(1000);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_1_1);
                sleep(350);
                drive.followTrajectory(trajectory_1_2);
                //arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                //drive to board
                drive.followTrajectory(trajectory_1_3);
                sleep(1000);
                //pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(1000);


            } else { // far from board
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_2_0);
                sleep(350);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_2_1);
                sleep(350);
                drive.followTrajectory(trajectory_2_2);
                sleep(350);
                //arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                drive.followTrajectory(trajectory_2_3);
                sleep(1000);
                //pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(1000);

            }
        }
    }
}
