package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Control (real)", group = "Iterative Opmode")
public class RealAutonomousControl extends OpMode {
    private MotorPair motors;


    @Override
    public void init() {
        motors = new MotorPair(
            hardwareMap.get(DcMotor.class, "left_drive"),
            hardwareMap.get(DcMotor.class, "right_drive")
        );
    }

    private void waitms(Runnable runnable, int delay){
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e){
                System.err.println(e);
            }
        }).start();
    }

    @Override
    public void start() {
        motors.setSpeed(0.5d);
        motors.setCounts(1000);

        waitms(() -> {
            motors.moveCounts(-500, 500);
        }, 2000);
    }

    @Override
    public void loop() {
        telemetry.addData("left target", motors.getLeftCountTarget());
        telemetry.addData("right target", motors.getRightCountTarget());

        telemetry.addData("left pos", motors.getLeftCounts());
        telemetry.addData("right pos", motors.getRightCounts());
    }
}
