package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Logger {
    public Telemetry telemetry;
    private List<String> logs;

    public Logger(Telemetry t) {
        telemetry = t;
        logs = new ArrayList<>();
    }

    public void clear() {
        logs = new ArrayList<>();
    }
    public void update() {
        if(logs == null) return;

        for(int i = 0; i < logs.size(); i++) {
            telemetry.addLine(logs.get(i));
        }
    }

    public void log(String text) {
        text = "["+(System.nanoTime()) + "ns] " + text;

        logs.add(text);
        System.out.println(text);
    }
}
