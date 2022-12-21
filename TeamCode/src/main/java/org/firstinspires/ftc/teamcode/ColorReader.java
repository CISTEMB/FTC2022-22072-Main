package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousPath;

import java.util.Arrays;
import java.util.Collections;

public class ColorReader {
    private RevColorSensorV3 sensor;

    public ColorReader(RevColorSensorV3 _sensor) {
        sensor = _sensor;
    }

    public String getColor() {
        return detectColor(sensor.red(), sensor.green(), sensor.blue());
    }

    private String detectColor(int red, int green, int blue) {
        Integer[] colors = { red, green, blue };
        Integer most = Collections.max(Arrays.asList(colors));
        if (red == most) {
            return "red";
        }

        if (green == most) {
            return "green";
        }

        if (blue == most) {
            return "blue";
        }

        return "unknown";
    }
}
