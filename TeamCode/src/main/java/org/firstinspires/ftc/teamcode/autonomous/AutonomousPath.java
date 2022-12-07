package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorPair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

class Instruction {
    public AutonomousInstruction instructionType;
    public double instructionAmount;
    public long delayTime;
    public Instruction(AutonomousInstruction type, double amount, long time) {
        instructionType = type;
        instructionAmount = amount;
        delayTime = time;
    }
}

public class AutonomousPath {
    public List<Instruction> events;
    private Runnable beforeEndCallback;
    private long executeBeforeEndCallbackBeforeEndTime;
    private Thread[] threads;

    public AutonomousPath() {
        events = new ArrayList<>();
    }
    public void execute(MotorPair wheels, DcMotor clawMotor, Servo clawServo, Runnable onComplete) {
        long currentTime = 0;

        threads = new Thread[events.size()];

        for(int i = 0; i < events.size(); i++) {
            Instruction instruction = events.get(i);
            currentTime += instruction.delayTime;

            final long currentThreadDelay = currentTime;

            Thread thread = new Thread(() -> {
                try {
                    Thread.sleep(currentThreadDelay);
                    executeInstruction(
                            instruction,
                            wheels,
                            clawMotor,
                            clawServo
                    );
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            });
            thread.start();

            threads[i] = thread;
        }

        final long totalTime = currentTime;
        new Thread(() -> {
            try {
                Thread.sleep(totalTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            onComplete.run();
        }).start();

        final long finalTime = executeBeforeEndCallbackBeforeEndTime;
        new Thread(() -> {
            try {
                Thread.sleep(30 * 1000 - finalTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            beforeEndCallback.run();
        }).start();
    }

    public void abort() {
        Arrays.stream(threads).iterator().forEachRemaining(v -> v.interrupt());
    }

    private void executeInstruction(Instruction instruction, MotorPair wheels, DcMotor clawMotor, Servo clawServo) {
        switch (instruction.instructionType) {
            case MOVE_FORWARD:
                wheels.setSpeed(0.5);
                wheels.moveCounts((int) instruction.instructionAmount);
                break;
            case TURN_RIGHT:
                wheels.setSpeed(0.5);
                wheels.moveCounts(-(int) instruction.instructionAmount, (int) instruction.instructionAmount);
                break;
            case TURN_LEFT:
                wheels.setSpeed(0.5);
                wheels.moveCounts((int) instruction.instructionAmount, -(int) instruction.instructionAmount);
                break;
            case MOVE_CLAW:
                clawMotor.setPower(1);
                clawMotor.setTargetPosition((int) instruction.instructionAmount);
                clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case GRAB_CLAW:
                clawServo.setPosition(1);
                break;
            case RELEASE_CLAW:
                clawServo.setPosition(0.7);
                break;
        }
    }

    public void scheduleMove(double counts, long delayTime) {
        events.add(new Instruction(AutonomousInstruction.MOVE_FORWARD, counts, delayTime));
    }
    public void scheduleClawMove(double counts, long delayTime) {
        events.add(new Instruction(AutonomousInstruction.MOVE_CLAW, counts, delayTime));
    }
    public void scheduleClawGrab(long delayTime) {
        events.add(new Instruction(AutonomousInstruction.GRAB_CLAW, 0, delayTime));
    }
    public void scheduleClawRelease(long delayTime) {
        events.add(new Instruction(AutonomousInstruction.RELEASE_CLAW, 0, delayTime));
    }
    public void scheduleLeftTurn(double counts, long delayTime) {
        events.add(new Instruction(AutonomousInstruction.TURN_LEFT, counts, delayTime));
    }
    public void scheduleRightTurn(double counts, long delayTime) {
        events.add(new Instruction(AutonomousInstruction.TURN_RIGHT, counts, delayTime));
    }

    public void onBeforeEnd(Runnable callback, long timeBeforeEnd) {
        beforeEndCallback = callback;
        executeBeforeEndCallbackBeforeEndTime = timeBeforeEnd;
    }
}
