package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.lang.Interpreter;
import org.firstinspires.ftc.teamcode.autonomous.lang.InterpreterScript;

import java.io.File;
import java.net.URL;

@Autonomous(name = "Autonomous Language", group = "Testing")
public class LangTestAutonomous extends OpMode {
    private final Interpreter interpreter = new Interpreter();
    private InterpreterScript script;

    @Override
    public void init() {
        //script = new InterpreterScript(source);
    }

    @Override
    public void loop() {

    }
}
