package org.firstinspires.ftc.teamcode.discoduckbots.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.DuckSensor;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelMechanism;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="forwardRedAutoGoated", group="Robot")
public class ForwardRedAutoGoated extends LinearOpMode {

    private SampleMecanumDrive sampleMecanumDrive = null;
    private Intake intake = null;

    private PixelMechanism pixelMechanism = null;
    private Arm arm = null;

    @Override
    public void runOpMode(){
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        sampleMecanumDrive = hardwareStore.getSampleMecanumDrive();
        arm = hardwareStore.getArm();
        pixelMechanism = hardwareStore.getPixelMechanism();



        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        sampleMecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Pose2d dropPixelPose = new Pose2d(30.3, -1.45, 0);
        Pose2d boardPose = new Pose2d(27.3, -35.8, 90);
        Pose2d parkPose = new Pose2d(1.7, -41.25, 90);

        Trajectory driveToPixel = sampleMecanumDrive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(dropPixelPose)
                .build();

        Trajectory driveToBoard = sampleMecanumDrive.trajectoryBuilder(driveToPixel.end())
                .lineToLinearHeading(boardPose)
                .build();

        Trajectory driveToPark = sampleMecanumDrive.trajectoryBuilder(driveToBoard.end())
                .lineToLinearHeading(parkPose)
                .build();

        pixelMechanism.closeGrabbers();

        waitForStart();
        if (opModeIsActive()) {

            pixelMechanism.toGrab(this);

            sleep(500);

            sampleMecanumDrive.followTrajectory(driveToPixel);

            sleep(1000);

            pixelMechanism.openLeftGrabber();
            pixelMechanism.toScore(this);

            sleep(1000);

            sampleMecanumDrive.followTrajectory(driveToBoard);

            arm.liftToPosition(-780, 0.5);
            pixelMechanism.openLeftGrabber();
            pixelMechanism.openRightGrabber();

            sleep( 500);

            arm.liftToPosition(0, 0.25);

            pixelMechanism.toGrab(this);

            sleep(1000);

            sampleMecanumDrive.followTrajectory(driveToPark);
        }
    }
}
