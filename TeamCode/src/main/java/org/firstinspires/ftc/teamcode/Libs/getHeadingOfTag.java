package org.firstinspires.ftc.teamcode.Libs;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class getHeadingOfTag {
    public static double getHeading(VisionPortal VisionPortalObj, AprilTagProcessor TagPro){
        double result = 0.0;
        ArrayList<AprilTagDetection> currentDetections = TagPro.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                result = detection.robotPose.getOrientation().getYaw();
            }
        }
        return result;
    }
}
