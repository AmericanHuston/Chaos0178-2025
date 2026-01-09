package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "RedDiagonal3", group = "Tests")
public class RedDiagonal3 extends Diagonal3 {
    public RedDiagonal3(){
        ConstantChaos.isRed = true;
        super.robot = new Robot3(ConstantChaos.Alliance.RED);
        super.StartingPose = ConstantChaos.RedStartingPoseDiagonal;
        super.ShootingPose = ConstantChaos.RedShootingPoseDiagonal;
        super.EndingPose = ConstantChaos.RedEndingPoseDiagonal;
    }
}