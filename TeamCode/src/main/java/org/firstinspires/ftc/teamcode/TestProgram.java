/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vec2F;


@TeleOp(name="TestProgram", group="Iterative Opmode")
//@Disabled
public class TestProgram extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Vec2F joystick = new Vec2F();

    private double leftMotorSpeed = 0.0d;
    private double rightMotorSpeed = 0.0d;
    private float robotSpeed = 1.0f;
    private float turnSensitivity = 1.0f;
    private float controllerDeadZone = 0.05f;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        Get references to the external motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

//        Set the motor rotation direction (Set left motor to reverse because they're pointing in opposite directions)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

//        Causes the motors to stop itself when no power is being applied
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        runtime.reset();

//        Reset motor speed
        setMotorSpeed(0.0d, 0.0d);
    }



    @Override
    public void loop() {
        float xPos = gamepad1.left_stick_x;
        float yPos = gamepad1.left_stick_y;

        double d = Math.sqrt(Math.pow(xPos, 2) + Math.pow(yPos, 2)); //Distance to joystick
        double ar = Math.atan2(yPos, xPos); //Angle radians
        double ad = ar * (180d / Math.PI); //Angle degrees

        double dx = 0d;
        double dy = 0d;

        if(ad >= 270 || ad < 90) {
//            Forward

            double turn = 0;
            if(ad >= 270) turn = 360d - ad;
            if(ad <  90 ) turn = ad;
            turn *= turnSensitivity;

            dx = turn / 90d;
            dy = d * robotSpeed;
        } else {
//            Backward

            double turn = ad - 180d;
            turn *= turnSensitivity;

            dx = turn / 90d;
            dy = -d * robotSpeed;
        }

        if(dx > controllerDeadZone) {
            setMotorSpeed(dy, dy * (1d - dx));
        } else if(dx < -controllerDeadZone) {
            setMotorSpeed(dy * (1d + dx), dy);
        } else if(Math.abs(dy) > controllerDeadZone) {
            setMotorSpeed(dy, dy);
        } else {
            setMotorSpeed(0d, 0d);
        }
    }

    @Override
    public void stop() {
        setMotorSpeed(0.0d, 0.0d);
    }

    private void updateJoystick() {


    }

    //    Sets the speed of both motors connected
    private void setMotorSpeed(double leftSpeed, double rightSpeed) {
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        leftMotorSpeed = leftSpeed;
        rightMotorSpeed = rightSpeed;
    }
}
