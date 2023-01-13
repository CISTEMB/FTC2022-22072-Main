package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "LEFT Autonomous Control (real)", group = "Iterative Opmode")
@Disabled
public class RealAutonomousControlLeft extends OpMode {
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
    }

    private void waitms(Runnable runnable, int delay){
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e){
                telemetry.addLine(String.format("Thread exception: {0}", e.getMessage()));
                System.err.println(e);
            }
        }).start();
    }

    @Override
    public void start() {

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftClaw.setPosition(0.7);

        waitms(() -> {
            telemetry.addLine("Stage 1");
            liftClaw.setPosition(1);
        }, 500);

        waitms(() -> {
            telemetry.addLine("Stage 2");
            liftMotor.setPower(1);
            liftMotor.setTargetPosition(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }, 1000);

        waitms(() -> {
            telemetry.addLine("Stage 3");
            motors.setSpeed(0.15);
            motors.moveCounts(1700, 1700);
        }, 4500);

        waitms(() -> {
            telemetry.addLine("Stage 4");
            motors.setSpeed(0.15);
            motors.moveCounts(-400, 400);
        }, 7500);

        waitms(() -> {
            telemetry.addLine("Stage 5");
            liftClaw.setPosition(0.7);
        }, 11500);
    }

    @Override
    public void loop() {
//        telemetry.addData("left target", motors.getLeftCountTarget());
//        telemetry.addData("right target", motors.getRightCountTarget());
//
//        telemetry.addData("left pos", motors.getLeftCounts());
//        telemetry.addData("right pos", motors.getRightCounts());
    }
}
