package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "BlueDiagonal3", group = "Tests")
public class BlueDiagonal3 extends Diagonal3 {
    public BlueDiagonal3(){
        ConstantChaos.isRed = false;
        super.robot = new Robot3(ConstantChaos.Alliance.BLUE);
        super.StartingPose = ConstantChaos.BlueStartingPoseDiagonal;
        super.ShootingPose = ConstantChaos.BlueShootingPoseDiagonal;
        super.EndingPose = ConstantChaos.BlueEndingPoseDiagonal;
    }
}