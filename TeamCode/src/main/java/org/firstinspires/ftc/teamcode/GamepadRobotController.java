package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GamepadRobotController {
    private final MotorPair motors;
    private MovementMode movementMode;

    private double forwardSpeed = 0.8;
    private double turnSpeed = 1.0;

    public GamepadRobotController(MotorPair motorPair) {
        motors = motorPair;
        movementMode = MovementMode.CAR;
    }

    public void setForwardSpeed(double speed) {
        forwardSpeed = speed;
    }


    public enum MovementMode { TANK, CAR }
    public MovementMode getTurnMode() {
        return movementMode;
    }
    public void setTurnMode(MovementMode movementMode) {
        this.movementMode = movementMode;
    }


    public void update(double xPos, double yPos, Telemetry telemetry) {

        double stickDistance = Math.sqrt(Math.pow(xPos, 2) + Math.pow(yPos, 2));
        double stickAngleRadians = Math.atan2(yPos, xPos);
        double stickAngleDegrees = stickAngleRadians * (180.0 / Math.PI);

        telemetry.addData("joyX", xPos);
        telemetry.addData("joyY", yPos);
        telemetry.addData("joyPow", stickDistance);
        telemetry.addData("joyDegrees", stickAngleDegrees);

        if(movementMode == MovementMode.CAR) carTurn(stickDistance, stickAngleRadians, stickAngleDegrees, telemetry);
        if(movementMode == MovementMode.TANK) tankTurn(xPos, yPos, telemetry);
    }

    private void tankTurn(double stickX, double stickY, Telemetry telemetry) {
        motors.setSpeed(
            stickX * -turnSpeed * forwardSpeed,
            stickX * turnSpeed * forwardSpeed
        );
    }

    private void carTurn(double stickDistance, double stickAngleRadians, double stickAngleDegrees, Telemetry telemetry) {

        double dx;
        double dy;
        double turn;

        if(stickAngleDegrees < 0) {
//            Forward

            turn = -stickAngleDegrees - 90.0;
            turn *= turnSpeed;

            dx = turn / 90.0;
            dy = stickDistance * forwardSpeed;
        } else {
//            Backward

            turn = (-stickAngleDegrees + 90.0);
            turn *= turnSpeed;

            dx = turn / 90.0;
            dy = -stickDistance * forwardSpeed;
        }

        telemetry.addData("hMove", dx);
        telemetry.addData("fMove", dy);
        telemetry.addData("turn", turn);

        if(dx > 0) {
//            Turn right
            motors.setSpeed(dy, dy * (1.0 - dx));

        } else if(dx < 0) {
//            Turn left
            motors.setSpeed(dy * (1.0 + dx), dy);

        } else if(Math.abs(dy) > 0) {
//            Move straight forward
            motors.setSpeed(dy, dy);

        } else {
//            Stop
            motors.setSpeed(0.0, 0.0);

        }
    }
}
