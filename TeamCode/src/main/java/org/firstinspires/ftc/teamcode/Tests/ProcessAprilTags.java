package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

class ProcessAprilTags{
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    /**
     * Only run this once, during init
     * @param hardwareMap Provide the hardware map for the processor to use
     */
    public void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    /**
     * Run this whenever you want to check for an april tag (And you can save on energy)
     * @return Returns the detection
     */
    public List<AprilTagDetection> telemetryAprilTag() {

        // Step through the list of detections and display info for each one.
        return aprilTag.getDetections();
    }
}
