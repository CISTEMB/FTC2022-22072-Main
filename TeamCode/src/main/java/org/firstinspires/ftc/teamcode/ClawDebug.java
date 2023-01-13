package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Debug", group = "Debug")
public class ClawDebug extends OpMode {

    private DcMotor liftMotor = null;
    private Servo liftClaw = null;

    @Override
    public void init() {
        HardwareGetter.get(hardwareMap);
        liftMotor = HardwareGetter.liftMotor;
        liftClaw = HardwareGetter.liftClaw;

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        liftMotor.setPower(-gamepad1.left_stick_y);

        liftClaw.setPosition(gamepad1.a ? 1 : 0.85);
    }
}
