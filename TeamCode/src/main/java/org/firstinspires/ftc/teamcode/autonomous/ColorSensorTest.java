package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorReader;
import org.firstinspires.ftc.teamcode.Maps;
import org.firstinspires.ftc.teamcode.MotorPair;

@Autonomous(name = "Cone on medium pole (left)", group = "Sequenced Autonomous")
public class ColorSensorTest extends OpMode {
    private final AutonomousPath path = new AutonomousPath();
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;
    private ColorReader colorReader;

    @Override
    public void init() {
        motors = new MotorPair(
                hardwareMap.get(DcMotor.class, Maps.LEFT_MOTOR),
                hardwareMap.get(DcMotor.class, Maps.RIGHT_MOTOR)
        );
        liftMotor = hardwareMap.get(DcMotor.class, "claw_lift");
        liftClaw = hardwareMap.get(Servo.class, "claw_grip");

        colorReader = new ColorReader(hardwareMap.get(RevColorSensorV3.class, "color_sensor"));

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setAutoClear(false);

        path.scheduleClawRelease(0);
        path.scheduleClawGrab(500);
        path.scheduleClawMove(2100, 500);
        path.scheduleMove(1600, 3500);
        path.scheduleLeftTurn(450, 3000);
        path.scheduleMove(100, 2000);
        path.scheduleClawRelease(500);
        path.scheduleMove(-100, 500);
        path.scheduleClawMove(0, 0);
   }

    @Override
    public void stop() {
        super.stop();
        path.abort();
    }

    @Override
    public void start() {
        path.execute(motors, liftMotor, liftClaw, () -> {
            telemetry.addLine("Execution finished");
        }, 1);
    }

    @Override
    public void loop() {

        //TODO:
        // drive forward until < 10 CM away on sensor
        // determine which color is detected
        // Green is spot one
        // Red is spot two
        // Blue is spot three
        // create three methods for each autonomous driving R G or B
        // start with spot one and estimate path -- first in pseudo code
        // spot two estimate path -- first in pseudo code
        // spot three estimate path -- first in pseudo code

//
//        int[] color = new int[3];
//
//        color[0] = sensor.red();
//        color[1] = sensor.green();
//        color[2] = sensor.blue();

//        int[] sortedColors = Arrays.copyOf(color, color.length);
//        Arrays.sort(sortedColors);
//        int maxColor = sortedColors[color.length - 1];

//        int index = Arrays.binarySearch(sortedColors, maxColor);


        /*
        telemetry.addData("isLightOn", sensor.isLightOn());
        telemetry.addData("Distance CM", sensor.getDistance(DistanceUnit.CM));

//        telemetry.addData("color", "Detected color " + index);
        telemetry.addData("red", sensor.red());
        telemetry.addData("green", sensor.green());
        telemetry.addData("blue", sensor.blue());

        telemetry.addData("detected color", detectColor(sensor.red(), sensor.green(), sensor.blue()));
        //telemetry.addData("color", "Absolute color [" + sensor.red() + ", " + sensor.green() + ", " + sensor.blue() + " ]");
        */
    }
}
