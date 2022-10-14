package org.firstinspires.ftc.teamcode;

public class AutonomousRobotController {
    private MotorPair motors;
    protected float currentX = 0f;
    protected float currentY = 0f;
    protected float currentRotation = 0f;

    private final float wheelDiameter = 9f;
    private final float wheelDistance = 36.7f;
    private final float motorRatio = 1f / 20f;
    private final float unitsPerRotation = (wheelDiameter * (float)Math.PI) * motorRatio;


    public AutonomousRobotController(MotorPair pair) {
        motors = pair;
    }

    private void moveForward(float distance, Runnable callback) {
        int rotations = Math.round(distance / unitsPerRotation);
        motors.moveCounts(rotations, rotations);

        motors.waitForMotorsToStop(() -> {

//        There will be some error because of float-to-int rounding, so we
//        take that into consideration when changing the position of the robot
            currentX += Math.sin(currentRotation) * rotations;
            currentY += Math.cos(currentRotation) * rotations;

            callback.run();
        });
    }
    private float mod(float a, float b) {
        return (a % b + b) % b;
    }
    private void rotate(float delta, Runnable callback) {
        delta = mod((float)(delta + Math.PI / 2), (float)(Math.PI * 2)) - (float)(Math.PI / 2);

        float wheelRotationDistance = ((float)Math.PI * wheelDistance) * delta / (2 * (float)Math.PI);
        int rotations = Math.round(wheelRotationDistance / unitsPerRotation);

        motors.waitForMotorsToStop(() -> {
            currentRotation += wheelRotationDistance;
            callback.run();
        });
    }

    private void moveX(float distance, Runnable callback) {
        float targetDirection = 0;
        if(distance > 0) {
            targetDirection = (float)(Math.PI *  (1d/2d));
        } else {
            targetDirection = (float)(Math.PI * -(1d/2d));
        }

        float turnDelta = currentRotation - targetDirection;
        rotate(turnDelta, () -> moveForward(distance, callback));
    }
    private void moveY(float distance, Runnable callback) {
        float targetDirection = 0;
        if(distance > 0) {
            targetDirection = 0f;
        } else {
            targetDirection = (float)Math.PI;
        }

        float turnDelta = currentRotation - targetDirection;
        rotate(turnDelta, () -> moveForward(distance, callback));
    }

    public void moveTo(float x, float y, Runnable callback) {
        float dx = currentX - x;
        float dy = currentY - y;

        moveX(dx, () -> moveY(dy, callback));
    }
}
