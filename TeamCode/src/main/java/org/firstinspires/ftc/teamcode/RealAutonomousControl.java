package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "Autonomous Control (real)", group = "Iterative Opmode")
@Disabled
public class RealAutonomousControl extends OpMode {
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;
    private ArrayList<Thread> threads;


    @Override
    public void init() {
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        liftMotor = hardwareMap.get(DcMotor.class, "claw_lift");
        liftClaw = hardwareMap.get(Servo.class, "claw_grip");

        threads = new ArrayList<>();

        telemetry.setAutoClear(false);
    }

    private void terminateThreads()
    {
        telemetry.addLine(String.format("Terminating {0} threads", String.valueOf(threads.size())));
        for (Thread thread : threads) {
            if (thread.isAlive() && !thread.isInterrupted())
            {
                thread.interrupt();
            }
        }
    }

    private void waitms(Runnable runnable, int delay){
        Thread t = new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (InterruptedException e){
                telemetry.addLine(String.format("Caught interrupted exception: {0}", e.getMessage()));
                terminateThreads();
            }
            catch (Exception e){
                telemetry.addLine(String.format("Caught exception: {0}", e.getMessage()));
                terminateThreads();
            }
        });

        t.start();
        threads.add(t);
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
            motors.moveCounts(400, -400);
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
