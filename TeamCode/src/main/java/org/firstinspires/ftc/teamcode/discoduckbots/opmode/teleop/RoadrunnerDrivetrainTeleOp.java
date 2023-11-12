package org.firstinspires.ftc.teamcode.discoduckbots.opmode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Arm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.Intake;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelGrabber;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Centerstage Roadrunner Opmode", group = "Linear Opmode")
public class RoadrunnerDrivetrainTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.75;
    private static double INTAKE_SPEED = 0.75;
    private static double OUTAKE_SPEED = 1.0;
    private static double SLIDE_SPEED = 0.6;
    private static double PIVOT_SPEED = 0.5;
    private static int SLIDE_POS_2 = -350;
    private static int SLIDE_RESET = 0;
    private static int PIVOT_POS_2 = 40;
    boolean liftAtEncoderPos = false;
    boolean pivotAtEncoderPos = false;


    private Intake intake = null;
    private DcMotor intakeMotor = null;
    private Arm arm = null;
    private DcMotor liftMotor = null;
    private DcMotor pivotMotor = null;
    private PixelGrabber pixelGrabber = null;
    private Servo grabberServo = null;
    private Servo wristServo = null;

    private boolean startLiftingPivotStage1 = false;
    private boolean startLiftingPivotStage2 = false;

    private int liftPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException{

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake = new Intake(intakeMotor);

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        arm = new Arm(liftMotor, pivotMotor);

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        pixelGrabber = new PixelGrabber(grabberServo, wristServo);

        waitForStart();

        while (opModeIsActive()) {

//            if (gamepad2.a){
//                liftAtEncoderPos = true;
//                arm.liftToPosition(1000, SLIDE_SPEED);
//            }
//            else if (gamepad2.dpad_up){
//                liftAtEncoderPos = false;
//                arm.lift(SLIDE_SPEED);
//            }
//            else if (gamepad2.dpad_down){
//                liftAtEncoderPos = false;
//                arm.lower(SLIDE_SPEED);
//            }
//            else if (!liftAtEncoderPos){
//                arm.stopLift();
//            }

            if (gamepad2.x){
                arm.pivotToPosition(100, PIVOT_SPEED);
            }

            if (gamepad2.y){
                arm.pivotToPosition(-100, PIVOT_SPEED);
            }

            if (gamepad2.dpad_up){
                arm.lift(SLIDE_SPEED);

                if (arm.getLiftPos() < 40){
                    arm.pivotToPosition(0, PIVOT_SPEED);
                    pixelGrabber.rotate(1.0);
                }
                else if (arm.getLiftPos() < 1000 && startLiftingPivotStage1 == false){
                    arm.pivotToPosition(10, PIVOT_SPEED);
                    pixelGrabber.rotate(1.0);
                    startLiftingPivotStage1 = true;
                }
                else if (arm.getLiftPos() < 1500 && startLiftingPivotStage2 == false) {
                    arm.pivotToPosition(-800, PIVOT_SPEED);
                    pixelGrabber.rotate(0.0);
                    startLiftingPivotStage2 = true;
                }

                liftPosition = arm.getLiftPos();

            } else {
                startLiftingPivotStage1 = false;
                startLiftingPivotStage2 = false;
                //arm.stopLift();
                if (liftPosition > 0) {
                    arm.liftToPosition(liftPosition, SLIDE_SPEED);
                }
            }

            Log.i("lift pos", ""+arm.getLiftPos());
            telemetry.addData("lift pos: ", arm.getLiftPos());
            telemetry.addData("wrist pos: " ,arm.getPivotPos());
            telemetry.addData("wristServo pos: ", wristServo.getPosition());
            telemetry.update();
        }
    }

}
