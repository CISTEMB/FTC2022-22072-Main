package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Autonomous Control", group="Iterative Opmode")
public class AutonomousControl extends OpMode {

    private float targetX = 0;
    private float targetY = 0;
    private float lastX = 0;
    private float lastY = 0;
    private boolean running = false;

    private GamepadController gamepadController;
    private AutonomousRobotController controller;
    private MotorPair motors;
    private Logger logger;

    private void addInstructions() {
        telemetry.addData("Move target", "dpad LRUD");
        telemetry.addData("Reset target", "Y");
        telemetry.addData("Execute path", "A");
//        telemetry.addData("Cancel path", "B");
        telemetry.addData("Clear Logs", "X");
    }
    @Override
    public void init() {
//        Get references to the external motors
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        controller = new AutonomousRobotController(motors);
        gamepadController = new GamepadController(gamepad1);
        logger = new Logger(telemetry);
        logger.clear();
        logger.log("Initializing");

        addInstructions();
        controller.setLogger(logger);
        motors.setLogger(logger);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        gamepadController.pressButtonA(() -> {
//            running = true;
//            lastX = targetX;
//            lastY = targetY;
//            controller.moveTo(targetX, targetY, 0.5f, () -> running = false);
//
//            logger.log("A button pressed");
//        });
//        gamepadController.pressButtonY(() -> {
//            targetX = lastX;
//            targetY = lastY;
//
//            logger.log("Y button pressed");
//        });
//        gamepadController.pressButtonX(() -> logger.clear());
//        gamepadController.pressButtonDL(() -> targetX -= 10);
//        gamepadController.pressButtonDR(() -> targetX += 10);
//        gamepadController.pressButtonDD(() -> targetY -= 10);
//        gamepadController.pressButtonDU(() -> targetY += 10);

        gamepadController.pressButtonDU(() -> controller.moveForward(100, 1, () -> {}));
        gamepadController.pressButtonDD(() -> controller.moveForward(-100, 1, () -> {}));

        logger.log("Init complete");
    }

    @Override
    public void start() {
        logger.log("Starting");
        motors.setSpeed(0.0d, 0.0d);

        logger.log("Start complete");
    }

    @Override
    public void loop() {
        gamepadController.update();

        addInstructions();
        telemetry.addData("Status", running ? "Executing Path" : "Awaiting Input");
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Left Speed", motors.leftMotorSpeed);
        telemetry.addData("Right Speed", motors.rightMotorSpeed);

        motors.debug(telemetry);
        logger.update();
    }
}