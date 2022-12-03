package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Control (backup)", group = "Iterative Opmode")
public class BackupAutonomousControl extends OpMode {
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;


    @Override
    public void init() {
        motors = new MotorPair(
            hardwareMap.get(DcMotor.class, "left_drive"),
            hardwareMap.get(DcMotor.class, "right_drive")
        );
    }

    @Override
    public void start() {
        motors.setSpeed(0.5, 0.5);
        motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

    }
}
