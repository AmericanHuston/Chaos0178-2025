package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.Robot3;

public class Jade extends LinearOpMode {

    Robot3 robot = new Robot3();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.setDesiredFlywheel(((double) 1 /6));
        robot.actMotors();
    }
}
