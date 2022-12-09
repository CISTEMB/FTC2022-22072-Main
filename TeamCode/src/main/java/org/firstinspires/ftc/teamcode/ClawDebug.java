package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Debug", group = "Debug")
public class ClawDebug extends OpMode {

    private DcMotor liftMotor = null;
    private Servo clawServo = null;

    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotor.class, "claw_lift");
        clawServo = hardwareMap.get(Servo.class, "claw_grip");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        liftMotor.setPower(-gamepad1.left_stick_y);

        clawServo.setPosition(gamepad1.a ? 1 : 0.7);
    }
}
