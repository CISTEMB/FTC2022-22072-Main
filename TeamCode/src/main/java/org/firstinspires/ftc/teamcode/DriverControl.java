package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Driver Control", group="Iterative Opmode")
public class DriverControl extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotController controller;
    private MotorPair motors;

    private float robotSpeed = 1.0f;
    private float turnSensitivity = 1.0f;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        Get references to the external motors
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
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
        float xPos = gamepad1.left_stick_x;
        float yPos = gamepad1.left_stick_y;
        controller.update(xPos, yPos, telemetry);
    }

    @Override
    public void stop() {
        motors.setSpeed(0.0d, 0.0d);
    }
}
