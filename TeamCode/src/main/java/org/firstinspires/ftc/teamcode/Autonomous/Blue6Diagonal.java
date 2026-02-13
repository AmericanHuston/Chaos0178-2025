package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "Blue6Diagonal", group = "Tests")
public class Blue6Diagonal extends Diagonal6 {
    public Blue6Diagonal(){
        ConstantChaos.isRed = false;//This is very important do not mix up or remove
        super.robot = new Robot3(ConstantChaos.Alliance.BLUE);
        super.StartingPose = ConstantChaos.BlueStartingPoseDiagonal;
        super.ShootingPose = ConstantChaos.BlueShootingPoseDiagonal;
        super.PickUpPartOne = ConstantChaos.BluePickUpPartOne;
        super.PickUpPartTwo = ConstantChaos.BluePickUpPartTwo;
        super.EndingPose = ConstantChaos.BlueEndingPoseDiagonal;
    }
}