package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorPair;

@Autonomous(name = "Autonomous Sequenced", group = "Iterative OpMode")
public class AutonomousSequencedController extends OpMode {
    private final AutonomousPath path = new AutonomousPath();
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;

    @Override
    public void init() {
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        liftMotor = hardwareMap.get(DcMotor.class, "claw_lift");
        liftClaw = hardwareMap.get(Servo.class, "claw_grip");

        telemetry.setAutoClear(false);

        path.scheduleClawRelease(0);
        path.scheduleClawGrab(500);
        path.scheduleClawMove(2100, 500);
        path.scheduleMove(1600, 3500);
        path.scheduleLeftTurn(450, 3000);
        path.scheduleMove(100, 2000);
        path.scheduleClawRelease(500);
        path.onBeforeEnd(() -> {
            liftMotor.setTargetPosition(0);
        }, 2000);
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
        });
    }

    @Override
    public void loop() {

    }
}
