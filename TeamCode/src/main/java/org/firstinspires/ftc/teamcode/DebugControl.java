package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Debug Control", group="Iterative Opmode")
@Disabled
public class DebugControl extends OpMode {

    private MotorPair motors;

    @Override
    public void init() {
        HardwareGetter.get(hardwareMap);
        motors = HardwareGetter.motors;
    }

    @Override
    public void loop() {
        motors.setSpeed(
                gamepad1.left_stick_y,
                gamepad1.right_stick_y
        );

        telemetry.addLine("\nJOYSTICK 1");
        telemetry.addData("gp1_LSx", gamepad1.left_stick_x);
        telemetry.addData("gp1_LSy", gamepad1.left_stick_y);
        telemetry.addData("gp1_RSx", gamepad1.right_stick_x);
        telemetry.addData("gp1_RSy", gamepad1.right_stick_y);

        telemetry.addLine("\nDPAD 1");
        telemetry.addData("gp1_DPu", gamepad1.dpad_up);
        telemetry.addData("gp1_DPd", gamepad1.dpad_down);
        telemetry.addData("gp1_DPl", gamepad1.dpad_left);
        telemetry.addData("gp1_DPr", gamepad1.dpad_right);

        telemetry.addLine("\nBUTTONS 1");
        telemetry.addData("gp1_A", gamepad1.a);
        telemetry.addData("gp1_B", gamepad1.b);
        telemetry.addData("gp1_X", gamepad1.x);
        telemetry.addData("gp1_y", gamepad1.y);

        telemetry.addLine("\nMOTORS");
        telemetry.addData("MTRL_PWR", motors.leftMotorPower);
        telemetry.addData("MTRR_PWR", motors.rightMotorPower);
        telemetry.addData("MTRL_POS", motors.getLeftCounts());
        telemetry.addData("MTRR_POS", motors.getRightCounts());

        telemetry.addLine("\nMOTOR RUN MODES");
        telemetry.addData("MTRL_RnT", motors.getLeftMotor().getMode());
        telemetry.addData("MTRR_RnT", motors.getRightMotor().getMode());
        telemetry.addData("MTRL_BkT", motors.getLeftMotor().getZeroPowerBehavior());
        telemetry.addData("MTRR_BkT", motors.getRightMotor().getZeroPowerBehavior());
    }
}