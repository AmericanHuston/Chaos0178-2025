package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "RedOffTheLine", group = "Tests")
public class RedOffTheLine extends OffTheLine {
    boolean isRed = ConstantChaos.isRed = true;
    public RedOffTheLine() {
        super.robot = new Robot3(ConstantChaos.Alliance.RED);
        super.StartingPose = ConstantChaos.RedStartingPoseOffTheLine;
        super.EndingPose = ConstantChaos.RedEndingPoseOffTheLine;
    }
}