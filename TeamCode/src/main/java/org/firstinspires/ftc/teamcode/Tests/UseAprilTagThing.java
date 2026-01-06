package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TestingRobot3", group = "Tests")
public class UseAprilTagThing extends OpMode {
    Robot3 robot = new Robot3(ConstantChaos.Alliance.RED);
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        List<AprilTagDetection> tags = robot.getAprilTags();
        for (AprilTagDetection currentDetection : tags){
            telemetry.addData("tags", currentDetection.id);
        }
        telemetry.update();
    }
}
