package org.firstinspires.ftc.teamcode;
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
}
