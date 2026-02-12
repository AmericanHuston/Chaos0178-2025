package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "BlueOffTheLine6", group = "Tests")
public class BlueOffTheLine6 extends OffTheLine6 {
    public BlueOffTheLine6() {
        ConstantChaos.isRed = false;//This is very important do not mix up or remove
        super.robot = new Robot3(ConstantChaos.Alliance.BLUE);
        super.StartingPose = ConstantChaos.BlueStartingPoseOffTheLine;
        super.ShootingPose = ConstantChaos.BlueShootingOffTheLine;
        super.FirstThree = ConstantChaos.BlueFirstThree;
        super.FirstThreePartOne = ConstantChaos.BlueFirstThreePartOne;
        super.EndingPose = ConstantChaos.BlueEndingPoseOffTheLine;

    }

}