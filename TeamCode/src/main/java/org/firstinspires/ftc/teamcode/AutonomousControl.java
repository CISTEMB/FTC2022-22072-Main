package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="AutonomousControl", group="Iterative Opmode")
public class AutonomousControl extends OpMode {

    private float targetX = 0;
    private float targetY = 0;
    private float lastX = 0;
    private float lastY = 0;
    private boolean running = false;

    private GamepadController gamepadController;
    private AutonomousRobotController controller;
    private MotorPair motors;

    private void addInstructions() {
        telemetry.addData("Move target", "dpad LRUD");
        telemetry.addData("Reset target", "Y");
        telemetry.addData("Execute path", "A");
//        telemetry.addData("Cancel path", "B");
    }
    @Override
    public void init() {
//        Get references to the external motors
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );

        controller = new AutonomousRobotController(motors);
        gamepadController = new GamepadController(gamepad1);

        addInstructions();

        gamepadController.pressButtonA(() -> {
            running = true;
            lastX = targetX;
            lastY = targetY;
            controller.moveTo(targetX, targetY, () -> running = false);
        });
        gamepadController.pressButtonY(() -> {
            targetX = lastX;
            targetY = lastY;
        });
        gamepadController.pressButtonDL(() -> targetX -= 10);
        gamepadController.pressButtonDR(() -> targetX += 10);
        gamepadController.pressButtonDD(() -> targetY -= 10);
        gamepadController.pressButtonDU(() -> targetY += 10);
    }

    @Override
    public void loop() {
        gamepadController.update();

        addInstructions();
        telemetry.addData("Status", running ? "Executing Path" : "Awaiting Input");
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
    }
}