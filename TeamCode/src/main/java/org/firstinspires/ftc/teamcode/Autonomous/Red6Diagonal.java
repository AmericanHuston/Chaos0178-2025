package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;

@Autonomous(name = "Red6Diagonal", group = "Tests")
public class Red6Diagonal extends Diagonal6 {
    public Red6Diagonal(){
        ConstantChaos.isRed = true;//This is very important do not mix up or remove
        super.robot = new Robot3(ConstantChaos.Alliance.RED);
        super.StartingPose = ConstantChaos.RedStartingPoseDiagonal;
        super.ShootingPose = ConstantChaos.RedShootingPoseDiagonal;
        super.PickUpPartOne = ConstantChaos.RedPickUpPartOne;
        super.PickUpPartTwo = ConstantChaos.RedPickUpPartTwo;
        super.EndingPose = ConstantChaos.RedEndingPoseDiagonal;
    }
}