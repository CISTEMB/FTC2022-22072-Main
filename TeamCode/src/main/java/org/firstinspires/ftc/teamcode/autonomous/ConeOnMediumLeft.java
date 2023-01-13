package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareGetter;
import org.firstinspires.ftc.teamcode.MotorPair;

@Autonomous(name = "Cone on medium pole (right)", group = "Sequenced Autonomous")
public class ConeOnMediumLeft extends OpMode {
    private final AutonomousPath path = new AutonomousPath();
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;

    @Override
    public void init() {
        HardwareGetter.get(hardwareMap);
        motors = HardwareGetter.motors;
        liftMotor = HardwareGetter.liftMotor;
        liftClaw = HardwareGetter.liftClaw;

        telemetry.setAutoClear(false);

        path.scheduleClawRelease(0);
        path.scheduleClawGrab(500);
        path.scheduleClawMove(2100, 500);
        path.scheduleMove(1600, 3500);
        path.scheduleRightTurn(450, 3000);
        path.scheduleMove(100, 2000);
        path.scheduleClawRelease(500);
        path.scheduleMove(-100, 500);
        path.scheduleClawMove(0, 0);
   }

    @Override
    public void stop() {
        super.stop();
        path.abort();
    }

    @Override
    public void start() {
        path.execute(motors, liftMotor, liftClaw, () -> {
            telemetry.addLine("Execution finished");
        }, 1);
    }

    @Override
    public void loop() {

    }
}
