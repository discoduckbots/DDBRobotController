package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.util.Encoder;

public class HardwareStore {
    private MecanumDrivetrain mecanumDrivetrain;
    private SampleMecanumDrive sampleMecanumDrive;
    private Arm arm;
    private PixelMechanism pixelMechanism;
    private FlipStateMachine flipStateMachine;

    //private DuckSensor duckSensor;

    //private IMU imu;
    //private TouchSensor touchSensor = null;
    //private WebcamName webcam = null;

    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    public DcMotor liftMotor;
    public DcMotor extensionMotor;
    //public DcMotor hangMotor;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public Servo leftHook;
    public Servo rightHook;
    public DcMotor flipMotor;

    //public DistanceSensor leftIntakeSensor;
    //public DistanceSensor rightIntakeSensor;

    //public RevBlinkinLedDriver ledDriver;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        //hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipStateMachine = new FlipStateMachine();

        //ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        //ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        arm = new Arm(liftMotor, extensionMotor);
        pixelMechanism = new PixelMechanism(flipMotor, rightHook, leftHook, rightGrabber, leftGrabber, flipStateMachine);


        // frontRight.setDirection(DcMotorEx.Direction.FORWARD);
       //  frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //turretSensor = hardwareMap.get(TouchSensor.class, "resetSensor");
        //distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        //distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        //duckSensor = new DuckSensor(distanceSensor1, distanceSensor2);

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

    //public DuckSensor getDuckSensor() { return duckSensor; }

    //public WebcamName getWebcam() {return webcam;}

    //public RevBlinkinLedDriver getLedDriver() { return ledDriver;}


    public Arm getArm() {
        return arm;
    }
    public PixelMechanism getPixelMechanism() {
        return pixelMechanism;
    }
    /*public DcMotor getHangMotor() {
        return hangMotor;
    }  */
    public FlipStateMachine getFlipStateMachine() {return flipStateMachine; }
}
