package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "ShootyShootyBangBang", group = "TeleOp")
public class ShootyShootyBangBang extends OpMode {

    private Follower follower;

    Robot3 robot = new Robot3();
    public Pose startingPose = new Pose(5,72, 0);
    public double flyVel = 0.0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);//new follower creator
        telemetry.addData("heading", follower.getHeading());
        follower.setStartingPose(startingPose);
        telemetry.addData("checking pose", startingPose);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update(); //MUST COME BEFORE SET TELE OP DRIVE
        follower.setTeleOpDrive(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        follower.updateDrivetrain();//Driving------------------

        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            follower.startTeleopDrive();
        }

        if (gamepad1.left_trigger > 0.01){
            follower.setTeleOpDrive(-gamepad1.left_stick_y/4, -gamepad1.left_stick_x/4, -gamepad1.right_stick_x/4, false);
            follower.update();
        }

        if (gamepad1.right_trigger > 0.01){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();
        }

        if (gamepad2.a && robot.getIsFlywheelOn()){
            robot.stopFlywheel();
        } else if (gamepad2.a && !robot.getIsFlywheelOn()) {
            robot.stopFlywheel();
        }
        if(gamepad2.dpadUpWasReleased()){
            flyVel = flyVel + 0.05;
            robot.spinFlywheel(flyVel);
        }
        if(gamepad2.dpad_right){
            flyVel = 0.5;
            robot.spinFlywheel(flyVel);
        }
        if(gamepad2.dpad_left){
            flyVel = 0.0;
            robot.spinFlywheel(flyVel);
        }
        if(gamepad2.dpadDownWasReleased()){
            flyVel = flyVel - 0.05;
            robot.spinFlywheel(flyVel);
        }
        if (gamepad2.b) {
            List<AprilTagDetection> currentDetections = robot.getAprilTags();
            if (currentDetections.isEmpty()) {
                telemetry.addData("AprilTagDetections", "No tags detectd");
            } else {
                telemetry.addData("AprilTagDetections", "Tags were detected");
                double headingOfTagFromRobot = robot.getHeading();
                Pose currentPose = follower.getPose();
                Path rotationPath = new Path(new BezierLine(currentPose, currentPose.setHeading(currentPose.getHeading() + headingOfTagFromRobot)));
                follower.followPath(rotationPath);
            }
        }
        //Driving----------------

        //Rewrite below----------
        if (gamepad1.back) {
            follower.setPose(new Pose(8, 71, Math.toRadians(0)));
            follower.update();
        }

        //Rewrite above----------

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("flyVel", flyVel);

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    @Override
    public void stop() {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}