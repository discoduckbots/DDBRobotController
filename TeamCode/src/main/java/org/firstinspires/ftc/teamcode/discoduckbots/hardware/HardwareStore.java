package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.util.Encoder;

public class HardwareStore {
    private MecanumDrivetrain mecanumDrivetrain;
    private SampleMecanumDrive sampleMecanumDrive;
    private Arm arm;
    private PixelGrabber pixelGrabber;
    private Intake intake;

    //private IMU imu;
    //private TouchSensor touchSensor = null;
    //private DistanceSensor distanceSensor = null;
    //private WebcamName webcam = null;

    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    public DcMotor intakeMotor;
    public DcMotor liftMotor;
    public DcMotor pivotMotor;
    public Servo grabberServo;
    public Servo wristServo;
    //public RevBlinkinLedDriver ledDriver;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        //ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        //ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        arm = new Arm(liftMotor, pivotMotor);
        pixelGrabber = new PixelGrabber(grabberServo, wristServo);
        intake = new Intake(intakeMotor);

        // frontRight.setDirection(DcMotorEx.Direction.FORWARD);
       //  frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //turretSensor = hardwareMap.get(TouchSensor.class, "resetSensor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        //BNO055IMU gyro = hardwareMap.get(BNO055IMU.class, "imu");
       //imu = new IMU(gyro);
       //imu.initialize();
        //hardwareMap.get(DcMotorEx.class, "leftEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hardwareMap.get(DcMotorEx.class, "rightEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hardwareMap.get(DcMotorEx.class, "backLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        mecanumDrivetrain = createDrivetrain(telemetry, opMode, frontLeft, frontRight, backLeft, backRight);
    }

    protected MecanumDrivetrain createDrivetrain(Telemetry telemetry,
                                 LinearOpMode opMode,
                                                 DcMotorEx frontLeft,
                                                 DcMotorEx frontRight,
                                                 DcMotorEx backLeft,
                                                 DcMotorEx backRight){
        return new MecanumDrivetrain(telemetry, opMode, frontLeft, frontRight, backLeft, backRight);
    }

    public MecanumDrivetrain getMecanumDrivetrain() {
        return mecanumDrivetrain;
    }

    public SampleMecanumDrive getSampleMecanumDrive() { return sampleMecanumDrive; }

    //public TouchSensor getArmStoppingSensor() { return armStoppingSensor; }
    //public DistanceSensor getDistanceSensor() { return distanceSensor; }

    //public WebcamName getWebcam() {return webcam;}

    //public RevBlinkinLedDriver getLedDriver() { return ledDriver;}

    public Intake getIntake() {
        return intake;
    }
    public Arm getArm() {
        return arm;
    }
    public PixelGrabber getPixelGrabber() {
        return pixelGrabber;
    }
   /* public DcMotor getPivotMotor() {
        return pivotMotor;
    }
    public DcMotor getLiftMotor() {
        return liftMotor;
    }
    public Servo getGrabberServo() {
        return grabberServo;
    }
    public Servo getWristServo() {
        return wristServo;
    } */
}
