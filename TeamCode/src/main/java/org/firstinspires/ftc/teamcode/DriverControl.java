package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Driver Control", group="Iterative Opmode")
public class DriverControl extends OpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private GamepadRobotController controller;
    private MotorPair motors;

    private final double defaultRobotAdjustment = 1.0; //100 percent of possible speed
    private final double sensitiveRobotAdjustment = 0.4; //80 percent of possible speed
    private final ToggleBoolean clawOpen = new ToggleBoolean(false);
    private final ToggleBoolean turnAround = new ToggleBoolean(false);

    private long lastSample = 0;
    private final double maxClawHeight = 2600;
    private final double minClawHeight = 0;
    private final int flipCounts = 1100;

    private DcMotor liftMotor = null;
    private Servo liftClaw = null;
    private TouchSensor resetSwitch = null;

    private boolean hasReset = false;
    private boolean canAdjustClaw = true;
    private boolean canMoveRobot = true;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        HardwareGetter.get(hardwareMap);
        motors = HardwareGetter.motors;
        liftMotor = HardwareGetter.liftMotor;
        liftClaw = HardwareGetter.liftClaw;
        resetSwitch = HardwareGetter.resetSwitch;

        controller = new GamepadRobotController(motors);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        runtime.reset();

        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        Reset motor speed
        motors.setSpeed(0.0d, 0.0d);
        motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawOpen.on("switchTrue", () -> liftClaw.setPosition(0.85));
        clawOpen.on("switchFalse", () -> liftClaw.setPosition(1));
        turnAround.on("switch", () -> flip());

        liftMotor.setPower(0.5);
    }

    @Override
    public void loop() {
        telemetry.addData("CLAW OPEN", clawOpen.getSwitch());
        telemetry.addData("MOVEMENT MODE", controller.getTurnMode().name());

        if(runtime.milliseconds() < 1000) {
            telemetry.addLine("Lifting claw for more leverage... ("+runtime.milliseconds()+")");
            liftClaw.setPosition(0.85d);
            return;
        }
        if (!hasReset) {
            telemetry.addLine("Waiting for reset to complete...");
            telemetry.addData("pressed", resetSwitch.isPressed());
            liftMotor.setPower(-0.8d);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (resetSwitch.isPressed()) hasReset = true;
            return;
        }

//        HAS NOT COMPILED YET
//        UNTESTED CODE: LED LIGHT, INIT RECALIBRATION

        float dt = (System.currentTimeMillis() - lastSample) / 1000f;
        lastSample = System.currentTimeMillis();
        bodyMovement();
        clawMovement(dt);

        telemetry.addData("resetSwitchHigh", resetSwitch.isPressed());
    }

    private void flip() {
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors.setSpeed(1);
        motors.setCounts(flipCounts, -flipCounts);
        motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        canMoveRobot = false;
    }

    private void bodyMovement() {
        float xPos = gamepad1.left_stick_x;
        float yPos = gamepad1.left_stick_y;
        if(xPos != 0 || yPos != 0) {
            canMoveRobot = true;
            motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(canMoveRobot) controller.update(xPos, yPos, telemetry);

        if(canMoveRobot) {
            if (gamepad1.y) {
                controller.setForwardSpeed(defaultRobotAdjustment);
            } else {
                controller.setForwardSpeed(sensitiveRobotAdjustment);
            }
            if (gamepad1.right_trigger > 1d / 3d) {
                controller.setTurnMode(GamepadRobotController.MovementMode.TANK);
            } else {
                controller.setTurnMode(GamepadRobotController.MovementMode.CAR);
            }
        }

        if(gamepad1.dpad_left) {
            motors.setSpeed(-controller.getForwardSpeed(), controller.getForwardSpeed());
        }
        if(gamepad1.dpad_right) {
            motors.setSpeed(controller.getForwardSpeed(), -controller.getForwardSpeed());
        }

        turnAround.set(gamepad1.x);

        telemetry.addData("Left Target", motors.getLeftCountTarget());
        telemetry.addData("Right Target", motors.getRightCountTarget());
        telemetry.addData("Left Pos", motors.getLeftCounts());
        telemetry.addData("Right Pos", motors.getRightCounts());
    }

    private void clawMovement(float dt) {

        if(resetSwitch.isPressed()) {
            telemetry.addLine("SWITCH HIGH; RESETTING ENCODER");
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        telemetry.addData("dt", dt);


        //Quick jump when stick is clicked
        if(gamepad1.right_stick_button) {
            int target = 0xdeadbeef;
            if(gamepad1.right_stick_y > 0) {
                target = (int)minClawHeight;
            }
            if(gamepad1.right_stick_y < 0) {
                target = (int)maxClawHeight;
            }
            if(target != 0xdeadbeef) {
                liftMotor.setTargetPosition(target);
                liftMotor.setPower(1);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                canAdjustClaw = false;
            }
        }

        clawOpen.set(gamepad1.a);


        if(gamepad1.right_stick_y < 0 && resetSwitch.isPressed()) {
            liftMotor.setPower(0);
        }

        if(
                gamepad1.right_stick_y != 0
                && canAdjustClaw
                && liftMotor.getCurrentPosition() < maxClawHeight + 10
                && liftMotor.getCurrentPosition() > minClawHeight - 10
        ) {
            liftMotor.setPower(-gamepad1.right_stick_y);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        if(gamepad1.right_stick_y == 0) canAdjustClaw = true;
        if(liftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            if(gamepad1.right_stick_y == 0) liftMotor.setPower(0);

            if(liftMotor.getCurrentPosition() >= maxClawHeight) {
                liftMotor.setPower(-0.1);
                telemetry.addLine("STOP! IMMINENT STRIPPING OF GEAR DETECTED");
            }
            if(liftMotor.getCurrentPosition() <= minClawHeight) {
                liftMotor.setPower(0.1);
            }
        }

        liftMotor.setZeroPowerBehavior(
            liftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER ?
            DcMotor.ZeroPowerBehavior.BRAKE :
            DcMotor.ZeroPowerBehavior.FLOAT
        );

        telemetry.addData("RJy", gamepad1.right_stick_y);
//        telemetry.addData("deltaClawMovement", deltaClawMovement);
        telemetry.addData("clawRealPos", liftMotor.getCurrentPosition());
        telemetry.addData("clawRealTarget", liftMotor.getTargetPosition());
    }

    @Override
    public void stop() {
        motors.setSpeed(0.0d, 0.0d);
    }
}
