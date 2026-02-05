package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "RedOffTheLine3", group = "Tests")
public class RedOffTheLine3 extends OffTheLine3 {
    public RedOffTheLine3() {
        ConstantChaos.isRed = true;//This is very important do not mix up or remove
        super.robot = new Robot3(ConstantChaos.Alliance.RED);
        super.StartingPose = ConstantChaos.RedStartingPoseOffTheLine;
        super.ShootingPose = ConstantChaos.RedShootingPoseOffTheLine;
        super.FirstThree = ConstantChaos.RedFirstThree;
        super.FirstThreePartOne = ConstantChaos.RedFirstThreePartOne;
        super.EndingPose = ConstantChaos.RedEndingPoseOffTheLine;

    }

}