package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorPair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

class Instruction {
    public AutonomousInstruction instructionType;
    public double instructionAmount;
    public long delayTime;
    public Instruction(AutonomousInstruction _instructionType, double _instructionAmount, long _delayTime) {
        instructionType = _instructionType;
        instructionAmount = _instructionAmount;
        delayTime = _delayTime;
    }
}
class IfBranch {
    public Supplier<Boolean> decider;
    public Consumer<InstructionBlock> ifTrue;
    public Consumer<InstructionBlock> ifFalse;
    public InstructionBlock trueBlock;
    public InstructionBlock falseBlock;
    public IfBranch(Supplier<Boolean> _decider, Consumer<InstructionBlock> _ifTrue, Consumer<InstructionBlock> _ifFalse) {
        decider = _decider;
        ifTrue = _ifTrue;
        ifFalse = _ifFalse;

        trueBlock = new InstructionBlock();
        falseBlock = new InstructionBlock();

        ifTrue.accept(trueBlock);
        ifFalse.accept(trueBlock);
    }
    public InstructionBlock decide() {
        if(decider.get()) {
            return trueBlock;
        } else {
            return falseBlock;
        }
    }
}

class InstructionBlock {
    public List<Object> events;
    private Thread[] threads;

    public InstructionBlock() {
        events = new ArrayList();
    }
    protected void execute(MotorPair wheels, DcMotor clawMotor, Servo clawServo, Runnable onComplete, double speedMultiplier, long timeOffset) {
        long currentTime = timeOffset;

        threads = new Thread[events.size()];

        for(int i = 0; i < events.size(); i++) {
            Object _instruction = events.get(i);
            if(_instruction instanceof Instruction) {
                Instruction instruction = (Instruction) _instruction;
                currentTime += instruction.delayTime;

                final long currentThreadDelay = currentTime;
                final double mult = speedMultiplier;

                Thread thread = new Thread(() -> {
                    try {
                        Thread.sleep((long) (currentThreadDelay / mult));
                        executeInstruction(
                                instruction,
                                wheels,
                                clawMotor,
                                clawServo,
                                speedMultiplier
                        );
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                });
                thread.start();

                threads[i] = thread;
            } else if(_instruction instanceof IfBranch) {
                IfBranch branch = (IfBranch) _instruction;
                InstructionBlock block = branch.decide();
                block.execute(wheels, clawMotor, clawServo, onComplete, speedMultiplier, currentTime);
            }
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
    }

    public void abort() {
        Arrays.stream(threads).iterator().forEachRemaining(v -> v.interrupt());
    }

    private void executeInstruction(Instruction instruction, MotorPair wheels, DcMotor clawMotor, Servo clawServo, double speedMultiplier) {
        switch (instruction.instructionType) {
            case MOVE_FORWARD:
                wheels.setSpeed(0.5 * speedMultiplier);
                wheels.moveCounts((int) instruction.instructionAmount);
                break;
            case TURN_RIGHT:
                wheels.setSpeed(0.5 * speedMultiplier);
                wheels.moveCounts((int) instruction.instructionAmount, -(int) instruction.instructionAmount);
                break;
            case TURN_LEFT:
                wheels.setSpeed(0.5 * speedMultiplier);
                wheels.moveCounts(-(int) instruction.instructionAmount, (int) instruction.instructionAmount);
                break;
            case MOVE_CLAW:
                clawMotor.setPower(1 * speedMultiplier);
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

    public void ifBranch(Supplier<Boolean> decider, Consumer<InstructionBlock> ifTrue, Consumer<InstructionBlock> ifFalse) {
        events.add(new IfBranch(decider, ifTrue, ifFalse));
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
}

public class AutonomousPath extends InstructionBlock {
    private long executeBeforeEndCallbackBeforeEndTime;
    private Runnable beforeEndCallback;

    public void execute(MotorPair wheels, DcMotor clawMotor, Servo clawServo, Runnable onComplete, double speedMultiplier) {
        super.execute(wheels, clawMotor, clawServo, onComplete, speedMultiplier, 0);


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

    public void onBeforeEnd(Runnable callback, long timeBeforeEnd) {
        beforeEndCallback = callback;
        executeBeforeEndCallbackBeforeEndTime = timeBeforeEnd;
    }
}