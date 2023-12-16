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
@Autonomous(name="BlueLeft", group="Robot")
public class BlueLeftAutonomous extends LinearOpMode{

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
        /*//far old
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(25.02307299221072  ,-1.6323970010151758 ,4.826679289444172 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(34.05269148749113  ,39.21633051623326 ,4.685907491092916 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(3.0932097021644545  ,34.47346962790206 ,4.685328976853096 ), velocityConstraint, accelerationConstraint).build();
*/
        //new far
        Trajectory trajectory_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(18.667034610285963  ,-0.6537097180759388 ,6.213570760323712 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(23.98363278081527  ,-4.246579240168563 ,5.2553583411356755 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(36.81764699237648  ,41.60417567856805 ,4.756486228348475 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(3.614473808357957  ,36.10594006704251 ,4.706734003725707 ), velocityConstraint, accelerationConstraint).build();

        /*//center old
        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(29.01983435553843  ,0.6159402153823778 ,0.018319617593649973 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(23.21095741253507  ,0.28597339084468903 ,0.016584074874249843 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(26.901732509020526  ,39.39505887225584 ,4.729296059077911 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(2.888611744260757  ,35.11650332889798 ,4.743180400833097 ), velocityConstraint, accelerationConstraint).build();
*/
        //center new
        Trajectory trajectory_1_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(28.278083169453758  ,0.5734465102913974 ,0.005592304318059682 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(19.561628084013254  ,0.5677836180401826 ,6.273929079342792 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(25.9700963022727  ,42.10221296826082 ,4.738552286914697 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_1_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(1.5287300255542782  ,35.6758692862758 ,4.728524706758151 ), velocityConstraint, accelerationConstraint).build();

        /*//close old
        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(25.132896597351902  ,4.208402079906273 ,0.767109881974231 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(14.0987834775114  ,2.565614006193363 ,0.935071849596044 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(16.963410363055445  ,37.91843733826426 ,4.724532958363216 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(2.669060205122779  ,35.20350125321692 ,4.722411739483938 ), velocityConstraint, accelerationConstraint).build();
*/
        //close new
        Trajectory trajectory_2_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(24.127177169389224  ,10.968615388828772 ,0.07038589917560678 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_1 = drive.trajectoryBuilder(trajectory_0.end()).lineToLinearHeading( new Pose2d(11.864743604892874  ,10.149931941917101 ,0.03586788286756004 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_2 = drive.trajectoryBuilder(trajectory_1.end()).lineToLinearHeading( new Pose2d(18.806554989271973  ,42.60918225740269 ,4.7437589150729025 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_2_3 = drive.trajectoryBuilder(trajectory_2.end()).lineToLinearHeading( new Pose2d(2.70750141300368  ,33.56216197820187 ,4.684943300693286 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.intakeLeft(pixelMechanism.LG_CLOSE_POS);
        pixelMechanism.hookRight(pixelMechanism.RH_CLOSE_POS);

        waitForStart();

        if (opModeIsActive()) {
            int duckPos = duckSensor.getDuckPos();
            if (duckPos == DuckSensorTensorFlow.CLOSER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_2_0);
                sleep(500);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(1000);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                arm.liftToPosition(arm.LIFT_ROW2, 0.5);
                sleep(2000);
                arm.extendToPosition(arm.EXTEND_OUT, 0.5);
                drive.followTrajectory(trajectory_2_1);
                sleep(2000);
                drive.followTrajectory(trajectory_2_2);
                sleep(1000);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(2000);
                drive.followTrajectory(trajectory_2_3 );
            } else if (duckPos == DuckSensorTensorFlow.CENTER) {
                pixelMechanism.flipToPosition(-124, 0.5);
                //drive forward
                drive.followTrajectory(trajectory_1_0);
                sleep(350);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                //drive backward
                drive.followTrajectory(trajectory_1_1);
                sleep(350);
                sleep(350);
                arm.liftToPosition(arm.LIFT_ROW2, 0.5);
                sleep(2000);
                arm.extendToPosition(arm.EXTEND_OUT, 0.5);
                sleep(2000);
                //drive to board
                drive.followTrajectory(trajectory_1_2);
                sleep(2000);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(2000);
                //drive to side
                drive.followTrajectory(trajectory_1_3);

            } else { // far from board
                pixelMechanism.flipToPosition(-124, 0.5);
                drive.followTrajectory(trajectory_0);
                sleep(350);
                drive.followTrajectory(trajectory_1);
                sleep(450);
                pixelMechanism.intakeLeft(pixelMechanism.LG_OPEN_POS);
                sleep(350);
                pixelMechanism.flipToPosition(0, 0.5);
                drive.followTrajectory(trajectory_2);
                sleep(350);
                arm.liftToPosition(arm.LIFT_ROW2, 0.5);
                sleep(350);
                arm.extendToPosition(arm.AUTOEXTEND, 0.5);
                sleep(2000);
                sleep(2000);
                pixelMechanism.hookRight(pixelMechanism.RH_OPEN_POS);
                sleep(2000);
                drive.followTrajectory(trajectory_2);
                sleep(2000);

            }
        }
    }
}
