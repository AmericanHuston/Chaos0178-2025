package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "April Tags Test", group = "tests")
public class AprilTagsTest extends OpMode {
    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();

        for (AprilTagDetection detection : currentDetections) {
            idsFound.append(detection.id);
            idsFound.append(' ');
            stop();
        }
        telemetry.addData("April Tags", idsFound);
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

    }
}

//        TagLibraryBuilder.addTag(21, "Pattern1", 3.5, DistanceUnit.INCH); //TODO Name this pattern
//        TagLibraryBuilder.addTag(22, "Pattern2", 3.5, DistanceUnit.INCH); //TODO Name this pattern
//        TagLibraryBuilder.addTag(23, "Pattern3", 3.5, DistanceUnit.INCH); //TODO Name this pattern
//        TagLibraryBuilder.addTag(20, "Blue Goal", 3.5, DistanceUnit.INCH);
//        TagLibraryBuilder.addTag(24, "Red Goal", 3.5, DistanceUnit.INCH);
