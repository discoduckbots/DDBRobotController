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
@Autonomous(name="BlueWing", group="Robot")
public class BlueWingAuto extends LinearOpMode{

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

        //trajectory_4_0 go forward
        //trajectory_4_1 turn to drop pixel
        //trajectory_4_2 go back to center position
        //trajectory_4_3 go forward towards the bridge
        //trajectory_4_4 go forward towards the board
        //trajectory_4_5 strafe over towards the board
        //trajectory_4_6 go forward to drop pixels
        //trajectory_4_7 move back a little
        //trajectory_4_8 go park

        Trajectory trajectory_4_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(23.20804529906632  ,-5.318597793835459 ,0.026013050668542093 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_1 = drive.trajectoryBuilder(trajectory_4_0.end()).lineToLinearHeading( new Pose2d(27.444982694620684  ,5.134166824678043 ,1.028280591133841 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_2 = drive.trajectoryBuilder(trajectory_4_1.end()).lineToLinearHeading( new Pose2d(23  ,-7.798672331328167 ,0.021231239883904074 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_3 = drive.trajectoryBuilder(trajectory_4_2.end()).lineToLinearHeading( new Pose2d(48.83762432845764  ,-9.452365546516003 ,4.714751369810249 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_4 = drive.trajectoryBuilder(trajectory_4_3.end()).lineToLinearHeading( new Pose2d(52.793119593444615  ,79.57010790957892 ,4.661195089022058 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_5 = drive.trajectoryBuilder(trajectory_4_4.end()).lineToLinearHeading( new Pose2d(19.390074879523226  ,83.98234331185033 ,4.588129020232447 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_6 = drive.trajectoryBuilder(trajectory_4_5.end()).lineToLinearHeading( new Pose2d(19.802599455636024  ,90.59783572453583 ,4.634225676196586 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_7 = drive.trajectoryBuilder(trajectory_4_6.end()).lineToLinearHeading( new Pose2d(19.342201091848786  ,85.02029905315588 ,4.621410423293694 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_4_8 = drive.trajectoryBuilder(trajectory_4_7.end()).lineToLinearHeading( new Pose2d(49.17564202440272  ,83.02122883805964 ,4.666168172238159 ), velocityConstraint, accelerationConstraint).build();

        //trajectory_5_0 go forward to drop pixel
        //trajectory_5_0_back back up to center
        //trajectory_5_1 turn away from tress
        //trajectory_5_2 strafe over in front to the bridge
        //trajectory_5_3 go forward towards the board
        //trajectory_5_4 strafe over to be in front of the board
        //trajectory_5_5 go forward a little to drop pixels
        //trajectory_5_6 move back a little bit
        //trajectory_5_7 go to park

        Trajectory trajectory_5_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(28.75  ,-1.3316905068534974 ,6.27285659588469 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_0_back = drive.trajectoryBuilder(trajectory_5_0.end()).back(18, velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_1 = drive.trajectoryBuilder(trajectory_5_0_back.end()).lineToLinearHeading( new Pose2d(19.49468956681412  ,-5 ,4.706717927691997 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_2 = drive.trajectoryBuilder(trajectory_5_1.end()).lineToLinearHeading( new Pose2d(52.163583986995704  ,-14.924575452942138 ,4.787243621305713 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_3 = drive.trajectoryBuilder(trajectory_5_2.end()).lineToLinearHeading( new Pose2d(48.184543588554455  ,81.3527213722567 ,4.800441419071408 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_4 = drive.trajectoryBuilder(trajectory_5_3.end()).lineToLinearHeading( new Pose2d(22.45185758237382  ,82.7054403645708 ,4.7218284497715395 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_5 = drive.trajectoryBuilder(trajectory_5_4.end()).lineToLinearHeading( new Pose2d(29.5  ,91 ,4.728905529732845 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_6 = drive.trajectoryBuilder(trajectory_5_5.end()).lineToLinearHeading( new Pose2d(23.069085212782753  ,81.39285525805877 ,4.706526655260607 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_5_7 = drive.trajectoryBuilder(trajectory_5_6.end()).lineToLinearHeading( new Pose2d(45.925757635041144  ,84.13423266428497 ,4.717237911418238 ), velocityConstraint, accelerationConstraint).build();

        //trajectory_6_0 go forward to drop pixel
        //trajectory_6_0_back back up after dropping pixel
        //trajectory_6_1 go back to the center
        //trajectory_6_2 go forward to in front of bridge
        //trajectory_6_3 turn in place
        //trajectory_6_4 go forward to the board
        //trajectory_6_5 go to board to drop pixels
        //trajectory_6_6 come back a little bit
        //trajectory_6_7 go park

        Trajectory trajectory_6_0 = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading( new Pose2d(24.71170732583952  ,-7.8249584050849785 ,5.7307905253353475 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_0_back = drive.trajectoryBuilder(trajectory_6_0.end()).back(8, velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_1 = drive.trajectoryBuilder(trajectory_6_0_back.end()).lineToLinearHeading( new Pose2d(12.498079415734226  ,2.5 ,6.2686486023942045 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_2 = drive.trajectoryBuilder(trajectory_6_1.end()).lineToLinearHeading( new Pose2d(48.96988985316221  ,-1.7614048772386375 ,6.21987413239064 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_3 = drive.trajectoryBuilder(trajectory_6_2.end()).lineToLinearHeading( new Pose2d(50.5  ,-2.8770466865477875 ,4.655648188511808 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_4 = drive.trajectoryBuilder(trajectory_6_3.end()).lineToLinearHeading( new Pose2d(50.5  ,71.17261691028655 ,4.751284404205062 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_5 = drive.trajectoryBuilder(trajectory_6_4.end()).lineToLinearHeading( new Pose2d(32.25500162104062  ,89.5 ,4.695624126671596 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_6 = drive.trajectoryBuilder(trajectory_6_5.end()).lineToLinearHeading( new Pose2d(31.815222261758127  ,81.36104240452825 ,4.67075871059135 ), velocityConstraint, accelerationConstraint).build();
        Trajectory trajectory_6_7 = drive.trajectoryBuilder(trajectory_6_6.end()).lineToLinearHeading( new Pose2d(48.80019593981383  ,83.22758134809689 ,4.69677176125991 ), velocityConstraint, accelerationConstraint).build();

        pixelMechanism.closeLeftGrabber();
        pixelMechanism.closeRightGrabber();
        waitForStart();


        if (opModeIsActive()) {
            if (duckSensor.getDuckPos() == CLOSER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_4_0);
                drive.followTrajectory(trajectory_4_1);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_4_2);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_4_3);
                drive.followTrajectory(trajectory_4_4);
                drive.followTrajectory(trajectory_4_5);
                arm.liftToPosition(arm.LIFT_HIGH_AUTO_POS, arm.LIFT_2_HIGH_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_4_6);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_4_7);
                pixelMechanism.closeRightGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_4_8);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if(duckSensor.getDuckPos() == CENTER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_5_0);
                pixelMechanism.openRightGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_5_0_back);
                drive.followTrajectory(trajectory_5_1);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_5_2);
                drive.followTrajectory(trajectory_5_3);
                drive.followTrajectory(trajectory_5_4);
                arm.liftToPosition(arm.LIFT_HIGH_AUTO_POS, arm.LIFT_2_HIGH_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_5_5);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_5_6);
                pixelMechanism.closeRightGrabber();
                arm.liftToPosition(0, 0, .5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_5_7);
                pixelMechanism.toInit(this);
                sleep(1000);
            }

            else if (duckSensor.getDuckPos() == FARTHER) {
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_6_0);
                pixelMechanism.openRightGrabber();
                drive.followTrajectory(trajectory_6_0_back);
                sleep(1000);
                drive.followTrajectory(trajectory_6_1);
                drive.followTrajectory(trajectory_6_2);
                drive.followTrajectory(trajectory_6_3);
                pixelMechanism.toScore(this);
                drive.followTrajectory(trajectory_6_4);
                arm.liftToPosition(arm.LIFT_HIGH_AUTO_POS, arm.LIFT_2_HIGH_AUTO_POS, arm.LIFT_POWER);
                drive.followTrajectory(trajectory_6_5);
                sleep(500);
                pixelMechanism.openLeftGrabber();
                sleep(1000);
                drive.followTrajectory(trajectory_6_6);
                pixelMechanism.closeLeftGrabber();
                arm.liftToPosition(0, 0,.5);
                pixelMechanism.toGrab(this);
                drive.followTrajectory(trajectory_6_7);
                //pixelMechanism.toInit(this);
                //sleep(1000);

            }
            pixelMechanism.flipToPosition(0, .5);
            while(pixelMechanism.flipMotor.getCurrentPosition() != 0) {
                sleep(1000);
            }

        }
    }
}