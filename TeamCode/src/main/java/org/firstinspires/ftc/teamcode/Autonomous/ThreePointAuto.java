package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "Autos", name = "ThreePointAuto")
public class ThreePointAuto extends LinearOpMode {
    Robot3 robot = new Robot3(ConstantChaos.Alliance.RED);

    private Follower follower;

    private final Pose StartingPose = new Pose(48,7, Math.toRadians(0));
    private final Pose EndingPose = new Pose(52,7, Math.toRadians(0));

    private PathChain Forward;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartingPose);

        Forward = follower.pathBuilder()
                .addPath(new BezierLine(StartingPose, EndingPose))
                .build();

        waitForStart();

        follower.followPath(Forward);

        robot.setLastPose(follower.getPose());
    }
}
