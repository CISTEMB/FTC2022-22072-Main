package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotController {
    private final MotorPair motors;

    public double forwardSpeed = 1.0;
    public double turnSpeed = 1.0;
    public RobotController(MotorPair motorPair) {
        motors = motorPair;
    }

    public void update(double xPos, double yPos, Telemetry telemetry) {

        double d = Math.sqrt(Math.pow(xPos, 2) + Math.pow(yPos, 2)); //Distance to joystick
        double ar = Math.atan2(yPos, xPos); //Angle radians
        double ad = ar * (180.0 / Math.PI); //Angle degrees

        telemetry.addData("joyX", xPos);
        telemetry.addData("joyY", yPos);
        telemetry.addData("joyPow", d);
        telemetry.addData("joyDegrees", ad);

        double dx;
        double dy;
        double turn;

        if(ad < 0) {
//            Forward

            turn = -ad - 90.0;
            turn *= turnSpeed;

            dx = turn / 90.0;
            dy = d * forwardSpeed;
        } else {
//            Backward

            turn = (-ad + 90.0);
            turn *= turnSpeed;

            dx = turn / 90.0;
            dy = -d * forwardSpeed;
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
