package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorPair;

public abstract class HardwareGetter {
    public static MotorPair motors;
    public static DcMotor liftMotor;
    public static Servo liftClaw;
    public static RevTouchSensor resetSwitch;

    public static void get(HardwareMap hardwareMap) {
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        liftMotor = hardwareMap.get(DcMotor.class, "claw_lift");
        liftClaw = hardwareMap.get(Servo.class, "claw_grip");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        resetSwitch = hardwareMap.get(RevTouchSensor.class, "reset_switch");
    }
}
