package org.firstinspires.ftc.teamcode.autonomous.lang;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.Arrays;
import java.util.Scanner;

public class InterpreterScript {
    public InterpreterInstruction[] instructions;

    public static InterpreterScript loadFromResource(String resourceId) throws FileNotFoundException {
        URL sourceURL = InterpreterScript.class.getClassLoader().getResource(resourceId);
        File file = null;
        try {
            file = new File(sourceURL.toURI());
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }

        String out = "";
        Scanner myReader = new Scanner(file);
        while (myReader.hasNextLine()) {
            out += myReader.nextLine() + "\n";
        }
        myReader.close();
        return new InterpreterScript(out);
    }

    public InterpreterScript(String source) {
        String[] lines = source.split(";");

        Arrays.stream(lines).map(line -> line.trim());

        Arrays.stream(lines).filter(line -> {
            if(line.length() == 0) return false;
            if(line.startsWith("#")) return false;
            return true;
        });

        instructions = new InterpreterInstruction[lines.length];
        for(int i = 0; i < lines.length; i++) {
            String line = lines[i];

            InterpreterInstruction instruction = new InterpreterInstruction(line);
            instructions[i] = instruction;
        }
    }
}
