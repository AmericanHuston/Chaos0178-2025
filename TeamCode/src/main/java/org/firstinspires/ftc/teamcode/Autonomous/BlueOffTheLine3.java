package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueOffTheLine3", group = "Tests")
public class BlueOffTheLine3 extends OffTheLine3 {
    public BlueOffTheLine3() {
        ConstantChaos.isRed = false;//This is very important do not mix up or remove
        super.robot = new Robot3(ConstantChaos.Alliance.BLUE);
        super.StartingPose = ConstantChaos.BlueStartingPoseOffTheLine;
        super.ShootingPose = ConstantChaos.BlueShootingOffTheLine;
        super.EndingPose = ConstantChaos.BlueEndingPoseOffTheLine;

    }

}