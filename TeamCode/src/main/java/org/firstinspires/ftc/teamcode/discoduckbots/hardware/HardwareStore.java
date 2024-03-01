package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.util.Encoder;

public class HardwareStore {
    private MecanumDrivetrain mecanumDrivetrain;
    private SampleMecanumDrive sampleMecanumDrive;
    private Arm arm;
    private PixelMechanism pixelMechanism;
    private DroneLauncher droneLauncher;
    private FlipStateMachine flipStateMachine;
    private PixelDetector pixelDetector;

    private TouchSensor pivotTouchSensor = null;
    public NormalizedColorSensor leftColorSensor;
    public NormalizedColorSensor rightColorSensor;
    public RevBlinkinLedDriver lights;
    //private WebcamName webcam = null;
    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public DcMotor flipMotor;
    public DcMotor pivotMotor;
    public Servo droneServo;



    //public RevBlinkinLedDriver ledDriver;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotTouchSensor = hardwareMap.get(TouchSensor.class, "pivotTouchSensor");
        leftColorSensor = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");

        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneServo = hardwareMap.get(Servo.class, "droneServo");


        flipStateMachine = new FlipStateMachine();

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        arm = new Arm(liftMotor1, liftMotor2);
        pixelMechanism = new PixelMechanism(flipMotor, pivotMotor, rightGrabber, leftGrabber, pivotTouchSensor, flipStateMachine);
        droneLauncher = new DroneLauncher(droneServo);
        pixelDetector = new PixelDetector(leftColorSensor, rightColorSensor);
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

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

    public PixelDetector getPixelDetector() { return pixelDetector; }




    //public WebcamName getWebcam() {return webcam;}

    public RevBlinkinLedDriver getLedDriver() { return lights;}


    public Arm getArm() {
        return arm;
    }
    public PixelMechanism getPixelMechanism() {
        return pixelMechanism;
    }
    public DroneLauncher getDroneLauncher() { return droneLauncher; }
    public FlipStateMachine getFlipStateMachine() {return flipStateMachine; }
}
