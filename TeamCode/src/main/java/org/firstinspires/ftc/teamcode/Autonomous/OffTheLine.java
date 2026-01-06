package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class OffTheLine extends OpMode {
    public Robot3 robot;
    private Follower follower;

    public Pose StartingPose;
    public Pose EndingPose;

    private PathChain Forward;

    int x = 1;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartingPose);

        Forward = follower.pathBuilder()
                .addPath(new BezierLine(StartingPose, EndingPose))
                .setLinearHeadingInterpolation(StartingPose.getHeading(), EndingPose.getHeading())
                .build();
    }

    @Override
    public void loop() {
        follower.update();
        robot.setLastPose(follower.getPose());
        if (x==1) {
            if (!follower.isBusy()){
                follower.followPath(Forward);
                x=x+1;
            }
        }
        robot.draw(follower);
    }
}