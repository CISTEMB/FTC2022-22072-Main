package org.firstinspires.ftc.teamcode;
import android.os.Handler;
import android.os.Looper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorPair {
    protected double leftMotorSpeed = 0d;
    protected double rightMotorSpeed = 0d;

    private DcMotor left;
    private DcMotor right;
    private Logger logger;

    public MotorPair(DcMotor leftMotor, DcMotor rightMotor) {
        left = leftMotor;
        right = rightMotor;

//        Set the motor rotation direction (Set left motor to reverse because they're pointing in opposite directions)
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

//        Causes the motors to stop itself when no power is being applied
    }

    private void tryLog(String text) {
        if(logger == null) return;
        logger.log(text);
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

    public void moveCounts(int leftCount, int rightCount) {
        left.setTargetPosition(left.getTargetPosition() + leftCount);
        right.setTargetPosition(right.getTargetPosition() + rightCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setCounts(int leftCount, int rightCount) {
        left.setTargetPosition(leftCount);
        right.setTargetPosition(rightCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("L_TGT", left.getTargetPosition());
        telemetry.addData("L_POS", left.getCurrentPosition());

        telemetry.addData("R_TGT", right.getTargetPosition());
        telemetry.addData("R_POS", right.getCurrentPosition());
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

    public void setMode(DcMotor.RunMode encoderMode) {
        left.setMode(encoderMode);
        right.setMode(encoderMode);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left.setZeroPowerBehavior(behavior);
        right.setZeroPowerBehavior(behavior);
    }

    public void setLogger(Logger _logger) {
        logger = _logger;
    }
}
