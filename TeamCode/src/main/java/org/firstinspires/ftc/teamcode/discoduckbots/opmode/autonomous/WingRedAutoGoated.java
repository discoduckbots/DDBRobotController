package org.firstinspires.ftc.teamcode.discoduckbots.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelGrabber;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="WingRedAutoGoated", group="Robot")
public class WingRedAutoGoated extends LinearOpMode {

    private SampleMecanumDrive sampleMecanumDrive = null;
    private Intake intake = null;
    private Arm arm = null;
    private PixelGrabber pixelGrabber = null;
    private static final double AUTONOMOUS_SPEED = 0.5;
    private static final double AUTONOMOUS_SPEED_SLOW = .3;
    private static final double LIFT_SPEED = 0.3;
    private static final double PIVOT_SPEED = 0.3;
    private static final double PIVOT_SPEED_RESET = .75;
    private static final int PIVOT_UP_LITTLE = 60;
    private static final int MOVE_LIFT_FOR_PUSH = 867;
    private static final int MOVE_LIFT_FOR_PIVOT = 800;
    private static final int PIVOT_GROUND = -546;
    private static final int PIVOT_CM = -675;
    private static final int PIVOT_BOARD = -1080;
    private static final double WRIST_GROUND = 0.53;

    @Override
    public void runOpMode(){
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        sampleMecanumDrive = hardwareStore.getSampleMecanumDrive();
        arm = hardwareStore.getArm();
        pixelGrabber = hardwareStore.getPixelGrabber();

        Arm arm = new Arm(hardwareStore.liftMotor, hardwareStore.pivotMotor);
        PixelGrabber pixelGrabber = new PixelGrabber(hardwareStore.grabberServo, hardwareStore.wristServo);
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        sampleMecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL * AUTONOMOUS_SPEED,
                DriveConstants.TRACK_WIDTH);waitForStart();
        TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * AUTONOMOUS_SPEED);

        TrajectoryVelocityConstraint slowVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL * AUTONOMOUS_SPEED_SLOW,
                DriveConstants.TRACK_WIDTH);waitForStart();
        TrajectoryAccelerationConstraint slowAccelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * AUTONOMOUS_SPEED_SLOW);


        Pose2d DUCK_POS = new Pose2d(-23.8, 1.4, Math.toRadians(0));
        Pose2d DRIVE_TO_BRIDGE = new Pose2d(-47, 1.4, Math.toRadians(0));
        Pose2d DRIVE_TO_PARK = new Pose2d(-47,38, Math.toRadians(90));

        Pose2d FORWARD_LITTLE = new Pose2d(-19.9, -1.0, Math.toRadians(0));
        Pose2d STRAFE_FROM_DUCK = new Pose2d(10, 10, Math.toRadians(0)); //diagonal??
        Pose2d DRIVE_UNDER_BRIDGE = new Pose2d(10, 10, Math.toRadians(0));
        Pose2d DRIVE_TO_BOARD = new Pose2d(-24.2, 38, Math.toRadians(90));
        Pose2d AWAY_FROM_BOARD = new Pose2d(-24.0, 27.5, Math.toRadians(90));



        Trajectory driveToDuck = sampleMecanumDrive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading( DUCK_POS,
                        slowVelocityConstraint, slowAccelerationConstraint)
                .build();

        Trajectory driveToFlipBridge = sampleMecanumDrive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading( DRIVE_TO_BRIDGE,
                        velocityConstraint, accelerationConstraint)
                .build();

        Trajectory driveToPark = sampleMecanumDrive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(DRIVE_TO_PARK,
                        velocityConstraint, accelerationConstraint)
                .build();

        Trajectory moveALittleForward = sampleMecanumDrive.trajectoryBuilder(driveToDuck.end())
                .lineToLinearHeading(FORWARD_LITTLE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory strafeFromDuck = sampleMecanumDrive.trajectoryBuilder(moveALittleForward.end())
                .lineToLinearHeading(STRAFE_FROM_DUCK, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory driveUnderBridge = sampleMecanumDrive.trajectoryBuilder(strafeFromDuck.end())
                .lineToLinearHeading(DRIVE_UNDER_BRIDGE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory driveToBoard = sampleMecanumDrive.trajectoryBuilder(moveALittleForward.end().plus(new Pose2d(0,0, Math.toRadians(90))))
                .lineToLinearHeading(DRIVE_TO_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory driveAwayFromBoard = sampleMecanumDrive.trajectoryBuilder(driveToBoard.end())
                .lineToLinearHeading(AWAY_FROM_BOARD, velocityConstraint, accelerationConstraint)
                .build();



        if (opModeIsActive()) {

            pixelGrabber.grab();
            sleep(250);
            arm.pivotToPosition(PIVOT_UP_LITTLE, PIVOT_SPEED);
            sampleMecanumDrive.followTrajectory(driveToDuck);
            arm.liftToPosition(MOVE_LIFT_FOR_PUSH, LIFT_SPEED);
            sleep(1000);
            //push pixel
            arm.pivotToPosition(PIVOT_GROUND, PIVOT_SPEED);
            sleep(600);
            sampleMecanumDrive.followTrajectory(driveToFlipBridge);
            sampleMecanumDrive.turn(Math.toRadians(90));
            sampleMecanumDrive.followTrajectory(driveToPark);
            //sampleMecanumDrive.followTrajectory(moveALittleForward);
            //sleep(500);
            /*sampleMecanumDrive.followTrajectory(strafeFromDuck);
            sleep(500);
            sampleMecanumDrive.followTrajectory(driveUnderBridge);
            sleep(500); //
            sampleMecanumDrive.turn(Math.toRadians(90));
            sleep(500);
            arm.pivotToPosition(PIVOT_BOARD, PIVOT_SPEED);
            sleep(250);
            pixelGrabber.rotate(0);
            sleep(500);
            sampleMecanumDrive.followTrajectory(driveToBoard);
            sleep(500);
            pixelGrabber.release();
            sleep(500);
            //place pixel
            sampleMecanumDrive.followTrajectory(driveAwayFromBoard);
            sleep(500);

             */
            arm.liftToPosition(MOVE_LIFT_FOR_PIVOT, LIFT_SPEED);
            sleep(1000);
            pixelGrabber.rotate(1);
            sleep(1000);
            arm.pivotToPosition(0, PIVOT_SPEED_RESET);
            sleep(300);
            arm.liftToPosition(0, LIFT_SPEED);
        }
    }
}
