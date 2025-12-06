package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "OffTheLine", group = "Tests")
public class OffTheLine extends OpMode {
    Robot3 robot = new Robot3();
    private Follower follower;

    private final Pose StartingPose = new Pose(48,7, Math.toRadians(0));
    private final Pose EndingPose = new Pose(52,7, Math.toRadians(0));

    private PathChain Forward;

    int x = 1;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartingPose);

        Forward = follower.pathBuilder()
                .addPath(new BezierLine(StartingPose, EndingPose))
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
    }
}