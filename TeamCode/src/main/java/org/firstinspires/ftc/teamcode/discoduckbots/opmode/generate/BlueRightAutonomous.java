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
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Autonomous(name="BlueRight", group="Robot")
public class BlueRightAutonomous extends LinearOpMode{

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
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(23.939402621490817  ,-8.183827650253622 ,5.809382144783726 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(3.663984276103438  ,-1.5488249384112152 ,4.67954383445516 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(6.179084137492377  ,53.95845387709959 ,4.656981779102999 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(36.49077013995197  ,84.74660139728778 ,4.702491565967232 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(28.753015902635457  ,0.8670257884277952 ,0.016776912954204803 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(3.1197067434487167  ,0.8616565844115662 ,4.752436628669916 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(2.938246986329252  ,51.764750092214896 ,4.694585204689975 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(25.843999275360865  ,87.47802014144273 ,4.731224439877302 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(21.64225935033772  ,0.7867594681671506 ,0.22581339160175862 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(24.919884696294844  ,4.316659698121112 ,0.8801129968150789 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(3.2589563528424392  ,-5.50657030644373 ,4.886844770383327 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(3.5390623307922544  ,50.104674724651915 ,4.677615453655825 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_4 = drive.trajectoryBuilder(trajectory_3.end()).lineToLinearHeading( new Pose2d(21.627082076536883  ,87.64627067437824 ,4.696320747409352 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.intakeLeft(pixelMechanism.LG_CLOSE_POS);
        pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);
        waitForStart();

        if (opModeIsActive()) {
            int duckPos = duckSensor.getDuckPos();
            if (duckPos == DuckSensorTensorFlow.CLOSER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_2_0);
                sleep(250);
                drive.followTrajectory(trajectory_2_1);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_2_2 );
                sleep(350);
                drive.followTrajectory(trajectory_2_3 );
                sleep(350);
                arm.extendToPosition(Arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                sleep(2000);
                drive.followTrajectory(trajectory_2_4 );
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);

            } else if (duckPos == DuckSensorTensorFlow.CENTER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                //drive forward
                drive.followTrajectory(trajectory_1_0);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_1_1);
                sleep(350);
                sleep(350);
                drive.followTrajectory(trajectory_1_2);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                //drive to board
                drive.followTrajectory(trajectory_1_3);
                sleep(350);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);
                //drive to side

            } else { // far from board
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_0);
                sleep(2000);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(2000);
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_1);
                sleep(350);
                drive.followTrajectory(trajectory_2);
                sleep(350);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                drive.followTrajectory(trajectory_3);
                sleep(1000);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(1000);

            }
        }
    }
}
