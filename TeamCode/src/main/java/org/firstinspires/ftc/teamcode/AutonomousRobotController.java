package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import java.util.Timer;
import java.util.TimerTask;

public class AutonomousRobotController {
    private MotorPair motors;
    protected float currentX = 0f;
    protected float currentY = 0f;
    protected float currentRotation = 0f;

//    Virtual "units" are centimeters irl
    private final float wheelDiameter = 9f;
    private final float wheelDistance = 36.7f;
//                             motor counts per rev / gear ratio
    private final float countsPerRev = (20f * 360f) / 1f;
    private final float unitsPerCount = ((float)Math.PI * wheelDiameter) / countsPerRev;
    private final float countsPerUnit = 1f / unitsPerCount;
    private Logger logger;


    /*

     */


    public AutonomousRobotController(MotorPair pair) {
        motors = pair;
    }

    private float mod(float a, float b) {
        return (a % b + b) % b;
    }
    public void setLogger(@Nullable Logger telemetry) {
        logger = telemetry;
    }
    private void tryLog(String text) {
        if(logger == null) return;
        logger.log(text);
    }

    public void waitForMotorStop(Runnable callback) {
        Timer timer = new Timer();
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                if(motors.isMoving()) return;

                timer.cancel();
                timer.purge();
                callback.run();
            }
        };

        timer.scheduleAtFixedRate(task,0,10);
    }

    public void moveForward(float distance, float speed, Runnable callback) {
        tryLog("moveForward " + distance);
        int rotations = Math.round(distance / countsPerUnit);
        motors.setSpeed(speed);
        motors.moveCounts(rotations);

        waitForMotorStop(() -> {
            motors.setSpeed(0.0, 0.0);

//        There will be some error because of float-to-int rounding, so we
//        take that into consideration when changing the position of the robot
            currentX += Math.sin(currentRotation) * rotations;
            currentY += Math.cos(currentRotation) * rotations;

            callback.run();
        });
    }
    public void rotate(float delta, float speed, Runnable callback) {
        tryLog("rotate " + delta);

        delta = mod((float)(delta + Math.PI / 2), (float)(Math.PI * 2)) - (float)(Math.PI / 2);

        float wheelRotationDistance = ((float)Math.PI * wheelDistance) * delta / (2 * (float)Math.PI);
        int rotations = Math.round(wheelRotationDistance / countsPerUnit);

        motors.setSpeed(speed);

        motors.moveCounts(rotations, -rotations);

        waitForMotorStop(() -> {
            motors.setSpeed(0.0, 0.0);
            currentRotation += wheelRotationDistance;
            callback.run();
        });
    }

    public void moveX(float distance, float speed, Runnable callback) {
        tryLog("moveX " + distance);

        float targetDirection = 0;
        if(distance > 0) {
            targetDirection = (float)(Math.PI *  (1.0/2.0));
        } else {
            targetDirection = (float)(Math.PI * -(1.0/2.0));
        }

        float turnDelta = currentRotation - targetDirection;
        rotate(turnDelta, speed, () -> moveForward(distance, speed, callback));
    }
    private void moveY(float distance, float speed, Runnable callback) {
        tryLog("moveY " + distance);

        float targetDirection = 0;
        if(distance > 0) {
            targetDirection = 0f;
        } else {
            targetDirection = (float)Math.PI;
        }

        float turnDelta = currentRotation - targetDirection;
        rotate(turnDelta, speed, () -> moveForward(distance, speed, callback));
    }

    public void moveTo(float x, float y, float speed, Runnable callback) {
        tryLog("moveTo (" + x + ", " + y + ")");
        float dx = currentX - x;
        float dy = currentY - y;

        moveX(dx, speed, () -> moveY(dy, speed, callback));
    }
}
