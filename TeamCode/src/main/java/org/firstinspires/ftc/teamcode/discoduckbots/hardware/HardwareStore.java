package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private FlipStateMachine flipStateMachine;

    //private DuckSensor duckSensor;
    private TouchSensor pivotTouchSensor = null;
    //private WebcamName webcam = null;
    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    public DcMotor liftMotor;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public DcMotor flipMotor;
    public DcMotor pivotMotor;


    //public RevBlinkinLedDriver ledDriver;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotTouchSensor = hardwareMap.get(TouchSensor.class, "pivotTouchSensor");

        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipStateMachine = new FlipStateMachine();

        //ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        //ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        arm = new Arm(liftMotor);
        pixelMechanism = new PixelMechanism(flipMotor, pivotMotor, rightGrabber, leftGrabber, pivotTouchSensor, flipStateMachine);

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

    //public DuckSensor getDuckSensor() { return duckSensor; }

    //public WebcamName getWebcam() {return webcam;}

    //public RevBlinkinLedDriver getLedDriver() { return ledDriver;}


    public Arm getArm() {
        return arm;
    }
    public PixelMechanism getPixelMechanism() {
        return pixelMechanism;
    }
    public FlipStateMachine getFlipStateMachine() {return flipStateMachine; }
}
