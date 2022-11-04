package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Driver Control", group="Iterative Opmode")
public class DriverControl extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotController controller;
    private MotorPair motors;

    private float robotSpeed = 1.0f;
    private float turnSensitivity = 1.0f;
    private float clawVerticalPosition = 0.0f;
    private float clawSpeed = 0.5f; //2 seconds to go up and down

    private long lastSample = 0;
    private double maxClawHeight = 2350;
    private double minClawHeight = 0;

    private double lerp(double v1, double v2, double t) {
        return v1 + (v2 - v1) * t;
    }

    private DcMotor liftMotor = null;
    private Servo clawServo = null;
    private TouchSensor resetSwitch = null;

    private boolean hasReset = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        Get references to the external motors
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        resetSwitch = hardwareMap.get(RevTouchSensor.class, "reset_switch");
        liftMotor = hardwareMap.get(DcMotor.class, "claw_lift");
        clawServo = hardwareMap.get(Servo.class, "claw_grip");
        controller = new RobotController(motors);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        runtime.reset();

//        Reset motor speed
        motors.setSpeed(0.0d, 0.0d);
    }

    @Override
    public void loop() {
        if (!hasReset) {
            telemetry.addLine("Waiting for reset to complete...");
            telemetry.addData("pressed", resetSwitch.isPressed());
            motors.setSpeed(0.2d, 0.2d);
            motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (resetSwitch.isPressed()) hasReset = true;
            return;
        }

        float dt = (System.currentTimeMillis() - lastSample) / 1000f;
        lastSample = System.currentTimeMillis();

        float xPos = gamepad1.left_stick_x;
        float yPos = gamepad1.left_stick_y;
        controller.update(xPos, yPos, telemetry);

        clawMovement(dt);
    }

    private void clawMovement(float dt) {

        if(resetSwitch.isPressed()) {
            motors.reset();
        }


        telemetry.addData("dt", dt);
        //Controller y position is inverted for some reason
        float deltaClawMovement = -(gamepad1.right_stick_y * dt * clawSpeed);

        clawVerticalPosition += deltaClawMovement;
        if(clawVerticalPosition > 1) clawVerticalPosition = 1.0f;
        if(clawVerticalPosition < 0) clawVerticalPosition = 0.0f;


        //Quick jump when stick is clicked
        if(gamepad1.left_stick_button) {
            if(gamepad1.right_stick_y > 0) {
                clawVerticalPosition = 0f;
            }
            if(gamepad1.right_stick_y < 0) {
                clawVerticalPosition = 1f;
            }
        }

        liftMotor.setPower(1.0f);
        liftMotor.setTargetPosition(-(int)lerp(minClawHeight, maxClawHeight, clawVerticalPosition));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(gamepad1.a) {
            clawServo.setPosition(1.0d);
        } else {
            clawServo.setPosition(0.0d);
        }

        telemetry.addData("RJy", gamepad1.right_stick_y);
        telemetry.addData("deltaClawMovement", deltaClawMovement);
        telemetry.addData("clawVerticalPosition", clawVerticalPosition);
        telemetry.addData("clawRealPos", liftMotor.getCurrentPosition());
        telemetry.addData("clawRealTarget", liftMotor.getTargetPosition());
    }

    @Override
    public void stop() {
        motors.setSpeed(0.0d, 0.0d);
    }
}
