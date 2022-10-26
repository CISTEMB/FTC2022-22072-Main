import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.Nullable;
import java.util.Timer;
import java.util.TimerTask;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;




@TeleOp(name="Autonomous Control", group="Iterative Opmode")
public class AutonomousControl extends OpMode {

    private float targetX = 0;
    private float targetY = 0;
    private float lastX = 0;
    private float lastY = 0;
    private boolean running = false;

    private GamepadController gamepadController;
    private AutonomousRobotController controller;
    private MotorPair motors;
    private Logger logger;

    private void addInstructions() {
        telemetry.addData("Move target", "dpad LRUD");
        telemetry.addData("Reset target", "Y");
        telemetry.addData("Execute path", "A");
//        telemetry.addData("Cancel path", "B");
        telemetry.addData("Clear Logs", "X");
    }
    @Override
    public void init() {
//        Get references to the external motors
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        controller = new AutonomousRobotController(motors);
        gamepadController = new GamepadController(gamepad1);
        logger = new Logger(telemetry);
        logger.clear();
        logger.log("Initializing");

        addInstructions();
        controller.setLogger(logger);
        motors.setLogger(logger);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        gamepadController.pressButtonA(() -> {
//            running = true;
//            lastX = targetX;
//            lastY = targetY;
//            controller.moveTo(targetX, targetY, 0.5f, () -> running = false);
//
//            logger.log("A button pressed");
//        });
//        gamepadController.pressButtonY(() -> {
//            targetX = lastX;
//            targetY = lastY;
//
//            logger.log("Y button pressed");
//        });
//        gamepadController.pressButtonX(() -> logger.clear());
//        gamepadController.pressButtonDL(() -> targetX -= 10);
//        gamepadController.pressButtonDR(() -> targetX += 10);
//        gamepadController.pressButtonDD(() -> targetY -= 10);
//        gamepadController.pressButtonDU(() -> targetY += 10);

        gamepadController.pressButtonDU(() -> controller.moveForward(100, 1, () -> {}));
        gamepadController.pressButtonDD(() -> controller.moveForward(-100, 1, () -> {}));
        gamepadController.pressButtonDL(() -> controller.moveForward(100, 1, () -> {}));
        gamepadController.pressButtonDR(() -> controller.moveForward(-100, 1, () -> {}));

        logger.log("Init complete");
    }

    @Override
    public void start() {
        logger.log("Starting");
        motors.setSpeed(0.0, 0.0);

        logger.log("Start complete");
    }

    @Override
    public void loop() {
        gamepadController.update();

        addInstructions();
        telemetry.addData("Status", running ? "Executing Path" : "Awaiting Input");
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Left Speed", motors.leftMotorPower);
        telemetry.addData("Right Speed", motors.rightMotorPower);

        motors.debug(telemetry);
        logger.update();
    }
}



public class AutonomousRobotController {
    private MotorPair motors;
    protected float currentX = 0f;
    protected float currentY = 0f;
    protected float currentRotation = 0f;

//    Virtual "units" are centimeters irl
    private final float wheelDiameter = 9f;
    private final float wheelDistance = 36.7f;
//                       counts per rev / gear ratio
    private final float countsPerRev = 20f;
    private final float unitsPerCount = ((float)Math.PI * wheelDiameter) / countsPerRev;
    private final float countsPerUnit = 1f / unitsPerCount;
    private Logger logger;


    public AutonomousRobotController(MotorPair pair) {
        motors = pair;
    }

    private float mod(float a, float b) {
        return (a % b + b) % b;
    }
    public void setLogger(@Nullable Logger telemetry) {
        logger = telemetry;
    }
    private void tryLog(String text) {
        if(logger == null) return;
        logger.log(text);
    }

    public void waitForMotorStop(Runnable callback) {
        Timer timer = new Timer();
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                if(motors.isMoving()) return;

                timer.cancel();
                timer.purge();
                callback.run();
            }
        };

        timer.scheduleAtFixedRate(task,0,10);
    }

    public void moveForward(float distance, float speed, Runnable callback) {
        tryLog("moveForward " + distance);
        int rotations = Math.round(distance / countsPerUnit);
        motors.setSpeed(speed);
        motors.moveCounts(rotations);

        waitForMotorStop(() -> {
            motors.setSpeed(0.0, 0.0);

//        There will be some error because of float-to-int rounding, so we
//        take that into consideration when changing the position of the robot
            currentX += Math.sin(currentRotation) * rotations;
            currentY += Math.cos(currentRotation) * rotations;

            callback.run();
        });
    }
    public void rotate(float delta, float speed, Runnable callback) {
        tryLog("rotate " + delta);

        delta = mod((float)(delta + Math.PI / 2), (float)(Math.PI * 2)) - (float)(Math.PI / 2);

        float wheelRotationDistance = ((float)Math.PI * wheelDistance) * delta / (2 * (float)Math.PI);
        int rotations = Math.round(wheelRotationDistance / countsPerUnit);

        motors.setSpeed(speed);

        motors.moveCounts(rotations, -rotations);

        waitForMotorStop(() -> {
            motors.setSpeed(0.0, 0.0);
            currentRotation += wheelRotationDistance;
            callback.run();
        });
    }

    public void moveX(float distance, float speed, Runnable callback) {
        tryLog("moveX " + distance);

        float targetDirection = 0;
        if(distance > 0) {
            targetDirection = (float)(Math.PI *  (1.0/2.0));
        } else {
            targetDirection = (float)(Math.PI * -(1.0/2.0));
        }

        float turnDelta = currentRotation - targetDirection;
        rotate(turnDelta, speed, () -> moveForward(distance, speed, callback));
    }
    private void moveY(float distance, float speed, Runnable callback) {
        tryLog("moveY " + distance);

        float targetDirection = 0;
        if(distance > 0) {
            targetDirection = 0f;
        } else {
            targetDirection = (float)Math.PI;
        }

        float turnDelta = currentRotation - targetDirection;
        rotate(turnDelta, speed, () -> moveForward(distance, speed, callback));
    }

    public void moveTo(float x, float y, float speed, Runnable callback) {
        tryLog("moveTo (" + x + ", " + y + ")");
        float dx = currentX - x;
        float dy = currentY - y;

        moveX(dx, speed, () -> moveY(dy, speed, callback));
    }
}




@TeleOp(name="Driver Control", group="Iterative Opmode")
public class DriverControl extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotController controller;
    private MotorPair motors;

    private float robotSpeed = 1.0f;
    private float turnSensitivity = 1.0f;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        Get references to the external motors
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, "left_drive"),
                hardwareMap.get(DcMotor.class, "right_drive")
        );
        controller = new RobotController(motors);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        runtime.reset();

//        Reset motor speed
        motors.setSpeed(0.0d, 0.0d);
    }

    @Override
    public void loop() {
        float xPos = gamepad1.left_stick_x;
        float yPos = gamepad1.left_stick_y;
        controller.update(xPos, yPos, telemetry);
    }

    @Override
    public void stop() {
        motors.setSpeed(0.0d, 0.0d);
    }
}



public class GamepadController {
    private Runnable callbackDownA;
    private Runnable callbackDownB;
    private Runnable callbackDownX;
    private Runnable callbackDownY;

    private Runnable callbackDownDU;
    private Runnable callbackDownDD;
    private Runnable callbackDownDL;
    private Runnable callbackDownDR;

    private Gamepad gamepad;

    public GamepadController(Gamepad newGamepad) {
        gamepad = newGamepad;
    }

    public void pressButtonA(Runnable callback) {
        callbackDownA = callback;
    }
    public void pressButtonB(Runnable callback) {
        callbackDownB = callback;
    }
    public void pressButtonX(Runnable callback) {
        callbackDownX = callback;
    }
    public void pressButtonY(Runnable callback) {
        callbackDownY = callback;
    }

    public void pressButtonDU(Runnable callback) {
        callbackDownDU = callback;
    }
    public void pressButtonDD(Runnable callback) {
        callbackDownDD = callback;
    }
    public void pressButtonDL(Runnable callback) {
        callbackDownDL = callback;
    }
    public void pressButtonDR(Runnable callback) {
        callbackDownDR = callback;
    }

    private boolean a = false;
    private boolean b = false;
    private boolean x = false;
    private boolean y = false;
    private boolean du = false;
    private boolean dd = false;
    private boolean dl = false;
    private boolean dr = false;
    public void update() {
        if(gamepad.a) {
            if(!a) {
                if (callbackDownA != null) callbackDownA.run();
                a = true;
            }
        } else {
            a = false;
        }

        if(gamepad.b) {
            if(!b) {
                if (callbackDownB != null) callbackDownB.run();
                b = true;
            }
        } else {
            b = false;
        }

        if(gamepad.x) {
            if(!x) {
                if (callbackDownX != null) callbackDownX.run();
                x = true;
            }
        } else {
            x = false;
        }

        if(gamepad.y) {
            if(!y) {
                if (callbackDownY != null) callbackDownY.run();
                y = true;
            }
        } else {
            y = false;
        }

        if(gamepad.dpad_up) {
            if(!du) {
                if (callbackDownDU != null) callbackDownDU.run();
                du = true;
            }
        } else {
            du = false;
        }

        if(gamepad.dpad_down) {
            if(!dd) {
                if (callbackDownDD != null) callbackDownDD.run();
                dd = true;
            }
        } else {
            dd = false;
        }

        if(gamepad.dpad_left) {
            if(!dl) {
                if (callbackDownDL != null) callbackDownDL.run();
                dl = true;
            }
        } else {
            dl = false;
        }

        if(gamepad.dpad_right) {
            if(!dr) {
                if (callbackDownDR != null) callbackDownDR.run();
                dr = true;
            }
        } else {
            dr = false;
        }
    }

}




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




public class MotorPair {
    protected double leftMotorPower = 0.0;
    protected double rightMotorPower = 0.0;

    private DcMotor left;
    private DcMotor right;
    private Logger logger;

    public MotorPair(DcMotor leftMotor, DcMotor rightMotor) {
        left = leftMotor;
        right = rightMotor;

//        Set the motor rotation direction (Set left motor to reverse because they're pointing in opposite directions)
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

//        Causes the motors to stop itself when no power is being applied
    }

    private void tryLog(String text) {
        if(logger == null) return;
        logger.log(text);
    }

    //    Sets the speed of both motors connected
    public void setSpeed(double leftPower, double rightPower) {
        if(leftPower > 1.0) leftPower = 1.0;
        if(leftPower < -1.0) leftPower = -1.0;
        if(rightPower > 1.0) rightPower = 1.0;
        if(rightPower < -1.0) rightPower = -1.0;

        left.setPower(leftPower);
        right.setPower(rightPower);

        leftMotorPower = leftPower;
        rightMotorPower = rightPower;
    }
    public void setSpeed(double power) {
        setSpeed(power, power);
    }

    public void moveCounts(int leftCount, int rightCount) {
        left.setTargetPosition(left.getTargetPosition() + leftCount);
        right.setTargetPosition(right.getTargetPosition() + rightCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveCounts(int counts) {
        moveCounts(counts, counts);
    }
    public void setCounts(int leftCount, int rightCount) {
        left.setTargetPosition(leftCount);
        right.setTargetPosition(rightCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setCounts(int counts) {
        setCounts(counts, counts);
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("L_TGT", left.getTargetPosition());
        telemetry.addData("L_POS", left.getCurrentPosition());

        telemetry.addData("R_TGT", right.getTargetPosition());
        telemetry.addData("R_POS", right.getCurrentPosition());
    }

    public int getLeftCounts() {
        return left.getCurrentPosition();
    }
    public int getRightCounts() {
        return right.getCurrentPosition();
    }

    public int getLeftCountTarget() {
        return left.getTargetPosition();
    }
    public int getRightCountTarget() {
        return right.getTargetPosition();
    }

    public boolean isMoving() {
        return left.isBusy() || right.isBusy();
    }
    public boolean isLeftMoving() {
        return left.isBusy();
    }
    public boolean isRightMoving() {
        return right.isBusy();
    }

    public void setMode(DcMotor.RunMode encoderMode) {
        left.setMode(encoderMode);
        right.setMode(encoderMode);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left.setZeroPowerBehavior(behavior);
        right.setZeroPowerBehavior(behavior);
    }

    public void setLogger(Logger _logger) {
        logger = _logger;
    }
}



public class RobotController {
    private final MotorPair motors;

    public double forwardSpeed = 1.0;
    public double turnSpeed = 1.0;
    public RobotController(MotorPair motorPair) {
        motors = motorPair;
    }

    public void update(double xPos, double yPos, Telemetry telemetry) {

        double d = Math.sqrt(Math.pow(xPos, 2) + Math.pow(yPos, 2)); //Distance to joystick
        double ar = Math.atan2(yPos, xPos); //Angle radians
        double ad = ar * (180.0 / Math.PI); //Angle degrees

        telemetry.addData("joyX", xPos);
        telemetry.addData("joyY", yPos);
        telemetry.addData("joyPow", d);
        telemetry.addData("joyDegrees", ad);

        double dx;
        double dy;
        double turn;

        if(ad < 0) {
//            Forward

            turn = -ad - 90.0;
            turn *= turnSpeed;

            dx = turn / 90.0;
            dy = d * forwardSpeed;
        } else {
//            Backward

            turn = (-ad + 90.0);
            turn *= turnSpeed;

            dx = turn / 90.0;
            dy = -d * forwardSpeed;
        }

        telemetry.addData("hMove", dx);
        telemetry.addData("fMove", dy);
        telemetry.addData("turn", turn);

        if(dx > 0) {
//            Turn right
            motors.setSpeed(dy, dy * (1.0 - dx));

        } else if(dx < 0) {
//            Turn left
            motors.setSpeed(dy * (1.0 + dx), dy);

        } else if(Math.abs(dy) > 0) {
//            Move straight forward
            motors.setSpeed(dy, dy);

        } else {
//            Stop
            motors.setSpeed(0.0, 0.0);

        }
    }
}

