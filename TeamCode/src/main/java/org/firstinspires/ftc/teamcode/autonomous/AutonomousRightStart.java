package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareGetter;
import org.firstinspires.ftc.teamcode.MotorPair;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Autonomous Right Start", group = "Sequenced Autonomous")
public class AutonomousRightStart extends LinearOpMode {
    private final AutonomousPath path = new AutonomousPath();

    private static final int ID_TAG_OF_INTEREST = 4;
    AprilTagDetection tagOfInterest = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;
    private ArrayList<Thread> threads;


    static final double FEET_PER_METER = 3.28084;
    static final int CODE_FOR_ZONE_ONE = 22;
    static final int CODE_FOR_ZONE_TWO = 137;
    static final int CODE_FOR_ZONE_THREE = 364;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareGetter.get(hardwareMap);

        motors = HardwareGetter.motors;
        liftMotor = HardwareGetter.liftMotor;
        liftClaw = HardwareGetter.liftClaw;

        threads = new ArrayList<>();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            telemetry.addLine("Driving to zone 2 -- no tag detected");
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {

            telemetry.addLine("Tag found, switching on " + tagOfInterest.id);
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            switch(tagOfInterest.id) {
                case CODE_FOR_ZONE_ONE:
                    driveToZoneOne();
                    break;
                case CODE_FOR_ZONE_TWO:
                    driveToZoneTwo();
                    break;
                case CODE_FOR_ZONE_THREE:
                    driveToZoneThree();
                    break;
            }
        }

        while (opModeIsActive()) {sleep(20);}
    }

    void driveToZoneOne() {
        telemetry.addLine("Driving to zone 1");
        //drive forward 2 feet
        motors.setSpeed(0.40);
        motors.moveCounts(1400, 1400);
        sleep(3000);
        //turn left 90 degrees
        motors.setSpeed(0.15);
        motors.moveCounts(555, -555);
        sleep(3000);
        // reverse for 20 inches
        motors.setSpeed(0.40);
        motors.moveCounts(-1200, -1200);
        sleep(3000);
    }

    void driveToZoneTwo() {
        telemetry.addLine("Driving to zone 2");
        //drive forward for 2 feet
        motors.setSpeed(0.40);
        motors.moveCounts(1400, 1400);
        sleep(3000);
    }

    void driveToZoneThree() {
        telemetry.addLine("Driving to zone 3");
        motors.setSpeed(0.40);
        //drive forward 2 feet
        motors.moveCounts(1380, 1380);
        sleep(3000);
        //turn left 90 degrees
        motors.setSpeed(0.15);
        motors.moveCounts(555, -555);
        sleep(3000);
        //move forwards 20 inches
        motors.setSpeed(0.40);
        motors.moveCounts(1180, 1180);
        sleep(3000);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
