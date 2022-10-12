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
    private void rotate(float delta, Runnable callback) {
        float wheelRotationDistance = ((float)Math.PI * wheelDistance) * delta / (2 * (float)Math.PI);
        int rotations = Math.round(wheelRotationDistance / unitsPerRotation);

        motors.waitForMotorsToStop(() -> {
            currentRotation += wheelRotationDistance;
            callback.run();
        });
    }

    public void moveTo(float x, float y) {
        float dx = currentX - x;
        float dy = currentY - y;

        float targetDirection = 0f;

        if(dx > 0) {
            targetDirection = (float)Math.PI / 2f;
        } else {
            targetDirection = (float)Math.PI / -2f;
        }
        if(dy > 0) {
            targetDirection += 0; //TODO
        }
    }
}
