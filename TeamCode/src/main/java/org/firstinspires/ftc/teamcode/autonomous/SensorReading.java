package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareGetter;
import org.firstinspires.ftc.teamcode.MotorPair;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Autonomous(name = "Sensor Reading 1", group = "Sequenced Autonomous")
public class SensorReading extends OpMode {
    protected int selectedAutonomousPath = -1;
    static final double FEET_PER_METER = 3.28084;

    private int webcamResourceId = -1;
    private AutonomousPath path;
    private MotorPair motors;
    private DcMotor liftMotor;
    private Servo liftClaw;
    private OpenCvWebcam camera;
    private AprilTagDetectionPipeline pipeline;
    private int highestTagId = 0;

    private HashMap<Integer, Float> tagsSeenDuringInitialization;

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

    protected void createPath(int conePath) {
        path = new AutonomousPath();

        path.scheduleLeftTurn(150, 1000);

        //path.scheduleMove(1200, 0);
//        switch (conePath) {
//            case 0:
//                path.scheduleLeftTurn(150, 1000);
//                break;
//            case 1:
//                break;
//            case 2:
//                path.scheduleRightTurn(150, 1000);
//                break;
//        }
        //path.scheduleMove(1200, 500);
    }

    @Override
    public void init() {
        HardwareGetter.get(hardwareMap);
        motors = HardwareGetter.motors;
        liftMotor = HardwareGetter.liftMotor;
        liftClaw = HardwareGetter.liftClaw;

        webcamResourceId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), webcamResourceId);
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to initialize! (error code " + errorCode + ")");
            }
        });

        readCamera(() -> {

            float highestTagScore = 0f;
            for(int id : tagsSeenDuringInitialization.keySet()) {
                float score = tagsSeenDuringInitialization.get(id);
                if(highestTagScore > score) {
                    highestTagId = id;
                    highestTagScore = score;
                }
            }


        }, 50);
    }

    @Override
    public void start() {
        //path.execute(motors, liftMotor, liftClaw, null, 0.80);
        createPath(getPathIdFromTagId(highestTagId));
    }

    private int getPathIdFromTagId(int tag) {
        switch (tag) {
            case 22/*left signal*/:
                return 0;
            case 137/*center signal*/:
                return 1;
            case 364/*right signal*/:
                return 2;
        }
        return -1;
    }

    private void readCamera(Runnable callback, int count) {

        Thread cameraChecker = new Thread(() -> {
            for(int i = 0; i < count; i++) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                readTags();
            }
            callback.run();
        });
        cameraChecker.start();
    }

    private void readTags() {
        List<AprilTagDetection> detections = pipeline.getLatestDetections();
        for(AprilTagDetection detection : detections) {
            if (detection == null) continue;

            tagToTelemetry(detection);
            int id = detection.id;
            Float score = tagsSeenDuringInitialization.get(id);
            if (score == null) score = 0f;

            score += detection.decisionMargin;
            tagsSeenDuringInitialization.put(id, score);
        }
    }

    @Override
    public void loop() {

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
