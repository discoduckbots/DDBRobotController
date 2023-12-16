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
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(25.02307299221072  ,-1.6323970010151758 ,4.826679289444172 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(34.05269148749113  ,39.21633051623326 ,4.685907491092916 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(3.0932097021644545  ,34.47346962790206 ,4.685328976853096 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(29.01983435553843  ,0.6159402153823778 ,0.018319617593649973 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(23.21095741253507  ,0.28597339084468903 ,0.016584074874249843 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(26.901732509020526  ,39.39505887225584 ,4.729296059077911 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(2.888611744260757  ,35.11650332889798 ,4.743180400833097 ), velocityConstraint, accelerationConstraint).build();

        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(25.132896597351902  ,4.208402079906273 ,0.767109881974231 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(14.0987834775114  ,2.565614006193363 ,0.935071849596044 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(16.963410363055445  ,37.91843733826426 ,4.724532958363216 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(2.669060205122779  ,35.20350125321692 ,4.722411739483938 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.intakeLeft(pixelMechanism.LG_CLOSE_POS);
        pixelMechanism.intakeRight(pixelMechanism.RG_CLOSE_POS);
        waitForStart();

        if (opModeIsActive()) {
            int duckPos = duckSensor.getDuckPos();
            if (duckPos == DuckSensorTensorFlow.CLOSER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_2_0 );
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_2_1 );
                pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);
                sleep(350);
                pixelMechanism.intakeRight(pixelMechanism.RG_HALF_POS);
                sleep(350);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                drive.followTrajectory(trajectory_2_2 );
                sleep(350);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);
                drive.followTrajectory(trajectory_2_3 );
            } else if (duckPos == DuckSensorTensorFlow.CENTER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                //drive forward
                drive.followTrajectory(trajectory_1_0);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_1_1);
                pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);
                sleep(350);
                pixelMechanism.intakeRight(pixelMechanism.RG_HALF_POS);
                sleep(350);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                //drive to board
                drive.followTrajectory(trajectory_1_2);

                sleep(350);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);
                //drive to side
                drive.followTrajectory(trajectory_1_3);

            } else { // far from board
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_0);
                sleep(350);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_1);
                pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);
                sleep(350);
                pixelMechanism.intakeRight(pixelMechanism.RG_HALF_POS);
                sleep(350);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                arm.liftToPosition(arm.LIFT_ROW1, 0.5);
                drive.followTrajectory(trajectory_2);
                sleep(350);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(350);

            }
        }
    }
}
