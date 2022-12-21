package org.firstinspires.ftc.teamcode.autonomous.lang;

import java.util.Arrays;

public class InterpreterInstruction {
    private final String name;
    private final String[] params;

    public InterpreterInstruction(String line) {
        String[] fullParams = line.split("\\s");

        name = fullParams[0];
        params = Arrays.copyOfRange(fullParams, 1, fullParams.length - 1);
    }
}
