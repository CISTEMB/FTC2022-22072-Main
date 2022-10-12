package org.firstinspires.ftc.teamcode;
import android.os.Handler;
import android.os.Looper;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorPair {
    protected double leftMotorSpeed = 0d;
    protected double rightMotorSpeed = 0d;

    private DcMotor left;
    private DcMotor right;

    public MotorPair(DcMotor leftMotor, DcMotor rightMotor) {
        left = leftMotor;
        right = rightMotor;

//        Set the motor rotation direction (Set left motor to reverse because they're pointing in opposite directions)
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

//        Causes the motors to stop itself when no power is being applied
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //    Sets the speed of both motors connected
    public void setSpeed(double leftSpeed, double rightSpeed) {
        if(leftSpeed > 1d) leftSpeed = 1d;
        if(leftSpeed < -1d) leftSpeed = -1d;
        if(rightSpeed > 1d) rightSpeed = 1d;
        if(rightSpeed < -1d) rightSpeed = -1d;

        left.setPower(leftSpeed);
        right.setPower(rightSpeed);

        leftMotorSpeed = leftSpeed;
        rightMotorSpeed = rightSpeed;
    }
    public void setSpeed(double leftSpeed, double rightSpeed, Telemetry telemetry) {
        setSpeed(leftSpeed, rightSpeed);
        telemetry.addData("leftMotor", leftSpeed);
        telemetry.addData("rightMotor", rightSpeed);
    }

    public void waitForMotorsToStop(Runnable callback) {
        final Handler handler = new Handler(Looper.getMainLooper());
        final Runnable runnable = () -> {
            if(isMoving()) return;
            callback.run();
            handler.removeCallbacksAndMessages(null);
        };

//        Hot reload to check if robot is stopped
        handler.postDelayed(runnable, 10);
    }
    public void moveCounts(int leftCount, int rightCount) {
        left.setTargetPosition(left.getTargetPosition() + leftCount);
        right.setTargetPosition(right.getTargetPosition() + rightCount);
    }
    public void setCounts(int leftCount, int rightCount) {
        left.setTargetPosition(leftCount);
        right.setTargetPosition(rightCount);
    }

    public int getLeftCounts() {
        return left.getCurrentPosition();
    }
    public int getRightCounts() {
        return right.getCurrentPosition();
    }

    public int getLeftCountTarget() {
        return left.getTargetPosition();
    }
    public int getRightCountTarget() {
        return right.getTargetPosition();
    }

    public boolean isMoving() {
        return left.isBusy() || right.isBusy();
    }
    public boolean isLeftMoving() {
        return left.isBusy();
    }
    public boolean isRightMoving() {
        return right.isBusy();
    }
}
