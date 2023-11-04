package org.firstinspires.ftc.teamcode.discoduckbots.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;

@Autonomous(name="CenterStage-Red", group="Robot")
public class RedAutonomous extends LinearOpMode {

    private MecanumDrivetrain mecanumDrivetrain = null;

    private static final double AUTONOUOMOUS_SPEED = 0.5;
    private static final double AUTONOUOMOUS_TIME = 2.0;

    @Override
    public void runOpMode(){
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            mecanumDrivetrain.strafeRightByTime(this, AUTONOUOMOUS_SPEED, AUTONOUOMOUS_TIME);
        }
    }
}
