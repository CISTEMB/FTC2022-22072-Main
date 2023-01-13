package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw Controller", group = "Iterative Opmode")
@Disabled
public class ClawController extends OpMode {

    private DcMotor liftMotor = null;
    private Servo liftClaw = null;

    @Override
    public void init() {
        HardwareGetter.get(hardwareMap);
        liftMotor = HardwareGetter.liftMotor;
        liftClaw = HardwareGetter.liftClaw;
//        ServoController.

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        clawServo.
    } //TODO: fix claw grip angle limits

    private double lerp(double v1, double v2, double t) {
        return v1 + (v2 - v1) * t;
    }

//    0
//    715
    @Override
    public void loop() {
        telemetry.addData("position", liftMotor.getCurrentPosition());

        double stickY = -(gamepad1.left_stick_y / 2d - 0.5d);
        int targetPos = (int)Math.round(lerp(0d, 715d, stickY));

        liftMotor.setPower(1.0d);
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(gamepad1.a) {
            liftClaw.setPosition(1.3d);
        } else {
            liftClaw.setPosition(0.5d);
        }
    }
}
