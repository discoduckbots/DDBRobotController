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
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelGrabber;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="forwardRedAutoGoated", group="Robot")
public class ForwardRedAutoGoated extends LinearOpMode {

    private SampleMecanumDrive sampleMecanumDrive = null;
    private Intake intake = null;
    private Arm arm = null;
    private PixelGrabber pixelGrabber = null;
    private DuckSensor duckSensor = null;
    private static final double AUTONOMOUS_SPEED = 0.5;
    private static final double AUTONOMOUS_SPEED_SLOW = .3;
    private static final double LIFT_SPEED = 0.3;
    private static final double PIVOT_SPEED = 0.7;
    private static final double PIVOT_SPEED_RESET = .75;
    private static final double OUTTAKE_SPEED = 0.3;
    private static final double OUTTAKE_SLOW = 0.2;
    private static final int PIVOT_UP_LITTLE = 60;
    private static final int MOVE_LIFT_FOR_PIVOT = 800;
    private static final int MOVE_LIFT_FOR_PUSH = 867;
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
        duckSensor = hardwareStore.getDuckSensor();
        intake = hardwareStore.getIntake();

        Arm arm = new Arm(hardwareStore.liftMotor, hardwareStore.pivotMotor);
        PixelGrabber pixelGrabber = new PixelGrabber(hardwareStore.grabberServo, hardwareStore.wristServo);
        Intake intake = new Intake(hardwareStore.intakeMotor);
        DuckSensor duckSensor = new DuckSensor(hardwareStore.distanceSensor1, hardwareStore.distanceSensor2);
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

                Pose2d DUCK_POS = new Pose2d(28.75, 3.77, Math.toRadians(0));

                Pose2d LEFT_OUTTAKE = new Pose2d(28.75, 5, Math.toRadians(0));
                Pose2d LEFT_TO_BOARD = new Pose2d(34.41, -33.89, Math.toRadians(90));
                Pose2d LEFT_AWAY_FROM_BOARD = new Pose2d(34.58, -26.51, Math.toRadians(90));

                Pose2d MIDDLE_PUSH_DUCK = new Pose2d(32.15, 3.52, Math.toRadians(0));
                Pose2d MIDDLE_OUTTAKE = new Pose2d(25, 3.52, Math.toRadians(0));
                Pose2d MIDDLE_BACK_LITTLE = new Pose2d(23, 3.52, Math.toRadians(0));
                Pose2d MIDDLE_TO_BOARD = new Pose2d(27.51, -33.89, Math.toRadians(90));
                Pose2d MIDDLE_AWAY_FROM_BOARD = new Pose2d(27.51, -26.51, Math.toRadians(90));

                Pose2d RIGHT_OUTTAKE = new Pose2d(18.31, -7.83, Math.toRadians(0));
                Pose2d RIGHT_BACK_LITTLE = new Pose2d(14.31,-6.8,Math.toRadians(0));
                Pose2d RIGHT_TO_BOARD = new Pose2d(18.59, -33.89, Math.toRadians(90));
                Pose2d RIGHT_AWAY_FROM_BOARD = new Pose2d(18.59, -26.51, Math.toRadians(90));

                Pose2d STRAFE = new Pose2d(-15, -26.51, Math.toRadians(90));



       Trajectory driveToDuck = sampleMecanumDrive.trajectoryBuilder(new Pose2d())

               .lineToLinearHeading( DUCK_POS, velocityConstraint, accelerationConstraint)
               .build();

       //left pos 1
        Trajectory leftOuttake = sampleMecanumDrive.trajectoryBuilder(driveToDuck.end())
                .lineToLinearHeading(LEFT_OUTTAKE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory leftMoveToBoard = sampleMecanumDrive.trajectoryBuilder(leftOuttake.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineToLinearHeading(LEFT_TO_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory leftAwayFromBoard = sampleMecanumDrive.trajectoryBuilder(leftMoveToBoard.end())
                .lineToLinearHeading(LEFT_AWAY_FROM_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory leftStrafe = sampleMecanumDrive.trajectoryBuilder(leftAwayFromBoard.end())
                .lineToLinearHeading(STRAFE, velocityConstraint, accelerationConstraint)
                .build();

        //middle pos 2
        Trajectory middlePushDuck = sampleMecanumDrive.trajectoryBuilder(driveToDuck.end())
                .lineToLinearHeading(MIDDLE_PUSH_DUCK, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory middleOutake = sampleMecanumDrive.trajectoryBuilder(middlePushDuck.end())
                .lineToLinearHeading(MIDDLE_OUTTAKE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory middleBackLittle = sampleMecanumDrive.trajectoryBuilder(middleOutake.end())
                .lineToLinearHeading(MIDDLE_BACK_LITTLE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory middleToBoard = sampleMecanumDrive.trajectoryBuilder(middleBackLittle.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineToLinearHeading(MIDDLE_TO_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory middleAwayFromBoard = sampleMecanumDrive.trajectoryBuilder(middleToBoard.end())
                .lineToLinearHeading(MIDDLE_AWAY_FROM_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory middleStrafe = sampleMecanumDrive.trajectoryBuilder(middleAwayFromBoard.end())
                .lineToLinearHeading(STRAFE, velocityConstraint, accelerationConstraint)
                .build();

        //right pos 3
        Trajectory rightOutake = sampleMecanumDrive.trajectoryBuilder(driveToDuck.end())
                .lineToLinearHeading(RIGHT_OUTTAKE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory rightBackLittle = sampleMecanumDrive.trajectoryBuilder(rightOutake.end())
                .lineToLinearHeading(RIGHT_BACK_LITTLE, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory rightToBoard = sampleMecanumDrive.trajectoryBuilder(rightBackLittle.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineToLinearHeading(RIGHT_TO_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory rightAwayFromBoard = sampleMecanumDrive.trajectoryBuilder(rightToBoard.end())
                .lineToLinearHeading(RIGHT_AWAY_FROM_BOARD, velocityConstraint, accelerationConstraint)
                .build();

        Trajectory rightStrafe = sampleMecanumDrive.trajectoryBuilder(rightAwayFromBoard.end())
                .lineToLinearHeading(STRAFE, velocityConstraint, accelerationConstraint)
                .build();




        if (opModeIsActive()) {

            pixelGrabber.grab();
            sleep(250);
            arm.pivotToPosition(PIVOT_UP_LITTLE, PIVOT_SPEED);
            sampleMecanumDrive.followTrajectory(driveToDuck);
            sleep(250);
            int duckPos = duckSensor.getDuckPos();
            sleep(1000);

            if (duckPos == 1) {
                sampleMecanumDrive.followTrajectory(leftOuttake);
                sleep(250);
                sampleMecanumDrive.turn(Math.toRadians(90));
                sleep(250);
                intake.outtake(OUTTAKE_SPEED);
                sleep(1000);
                intake.stop();
                sleep(250);
                arm.liftToPosition(MOVE_LIFT_FOR_PIVOT, LIFT_SPEED);
                sleep(1000);
                arm.pivotToPosition(PIVOT_BOARD, PIVOT_SPEED);
                sleep(250);
                pixelGrabber.rotate(0);
                sleep(250);
                sampleMecanumDrive.followTrajectory(leftMoveToBoard);
                sleep(250);
                pixelGrabber.release();
                sleep(1000);
                sampleMecanumDrive.followTrajectory(leftAwayFromBoard);
                sampleMecanumDrive.followTrajectory(leftStrafe);

            }
            else if (duckPos == 3) {
                sampleMecanumDrive.followTrajectory(rightOutake);
                sleep(250);
                intake.outtake(OUTTAKE_SPEED);
                sleep(1000);
                intake.stop();
                sleep(250);
                sampleMecanumDrive.followTrajectory(rightBackLittle);
                sleep(250);
                sampleMecanumDrive.turn(Math.toRadians(90));
                sleep(250);
                arm.liftToPosition(MOVE_LIFT_FOR_PIVOT, LIFT_SPEED);
                sleep(1000);
                arm.pivotToPosition(PIVOT_BOARD, PIVOT_SPEED);
                sleep(250);
                pixelGrabber.rotate(0);
                sleep(250);
                sampleMecanumDrive.followTrajectory(rightToBoard);
                sleep(250);
                pixelGrabber.release();
                sleep(1000);
                sampleMecanumDrive.followTrajectory(rightAwayFromBoard);
                sleep(1000);
                sampleMecanumDrive.followTrajectory(rightStrafe);
            }
            else {
                sampleMecanumDrive.followTrajectory(middlePushDuck);
                sleep(250);
                sampleMecanumDrive.followTrajectory(middleOutake);
                sleep(250);
                intake.outtake(OUTTAKE_SPEED);
                sleep(1000);
                intake.stop();
                sleep(250);
                sampleMecanumDrive.followTrajectory(middleBackLittle);
                sleep(250);
                sampleMecanumDrive.turn(Math.toRadians(90));
                sleep(250);
                arm.liftToPosition(MOVE_LIFT_FOR_PIVOT, LIFT_SPEED);
                sleep(1000);
                arm.pivotToPosition(PIVOT_BOARD, PIVOT_SPEED);
                sleep(250);
                pixelGrabber.rotate(0);
                sleep(250);
                sampleMecanumDrive.followTrajectory(middleToBoard);
                sleep(250);
                pixelGrabber.release();
                sleep(1000);
                sampleMecanumDrive.followTrajectory(middleAwayFromBoard);
                sampleMecanumDrive.followTrajectory(middleStrafe);

            }

         }
    }
}
