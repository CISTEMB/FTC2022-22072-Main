package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Debug Control Individual", group="Iterative Opmode")
@Disabled
public class DebugControlIndividual extends OpMode {
    GamepadController controller = new GamepadController(gamepad1);

    private final Map<String, Class> classes = new HashMap<>();
    private final Map<String, Integer> ports = new HashMap<>();

    private int selectedClass = 0;
    private int selectedPort = 0;

    @Override
    public void init() {
        telemetry.setAutoClear(false);

        classes.put("Motor", DcMotor.class);
        classes.put("Servo", Servo.class);
        classes.put("Button", RevTouchSensor.class);

        ports.put("Motor", 4);
        ports.put("Servo", 5);
        ports.put("Button", 4);
    }

    private int mod(int v, int m) {
        return ((v % m) + m) % m;
    }

    @Override
    public void loop() {
        controller.pressButtonDD(() -> {
            selectedClass = mod(selectedClass + 1, classes.size());
        });
        controller.pressButtonDU(() -> {
            selectedClass = mod(selectedClass - 1, classes.size());
        });
        controller.pressButtonDL(() -> {
//            selectedPort = mod(selectedPort - 1, ports.get(classes.keySet());
        });
        controller.pressButtonDR(() -> {
            selectedPort = mod(selectedPort + 1, classes.size());
        });
    }
}
