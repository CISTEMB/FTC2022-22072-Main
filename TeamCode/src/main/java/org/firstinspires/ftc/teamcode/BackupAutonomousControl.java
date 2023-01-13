package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Control (backup)", group = "Iterative Opmode")
@Disabled
public class BackupAutonomousControl extends OpMode {
    private MotorPair motors;


    @Override
    public void init() {
        HardwareGetter.get(hardwareMap);
        motors = HardwareGetter.motors;
    }

    @Override
    public void start() {
        motors.setSpeed(0.5, 0.5);
        motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

    }
}
