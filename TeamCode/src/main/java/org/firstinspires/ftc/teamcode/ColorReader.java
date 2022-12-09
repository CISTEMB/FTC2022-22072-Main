package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "Color Reader Test", group = "Iterative OpMode")
public class ColorReader extends OpMode {
    private RevColorSensorV3 sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
    }

    @Override
    public void loop() {
        sensor.enableLed(true);

        int[] color = new int[3];

        color[0] = sensor.red();
        color[1] = sensor.green();
        color[2] = sensor.blue();

        int[] sortedColors = Arrays.copyOf(color, color.length);
        Arrays.sort(sortedColors);
        int maxColor = sortedColors[color.length - 1];

        int index = Arrays.binarySearch(sortedColors, maxColor);


        telemetry.addData("color", "Detected color " + index);
        telemetry.addData("color", "Absolute color [" + sensor.red() + ", " + sensor.green() + ", " + sensor.blue() + " ]");
    }
}
