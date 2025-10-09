package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.Robot3;

@TeleOp(name = "FlywheelAngleTesting", group = "tests")
public class FlywheelAngleTesting extends LinearOpMode {
    enum flywheelAdjustMode {
        DPAD,
        TRIGGER
    }
    flywheelAdjustMode mode = flywheelAdjustMode.DPAD;
    double flywheelSpeed = 0.00;
    Robot3 robot = new Robot3();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (mode == flywheelAdjustMode.TRIGGER) {
                    mode = flywheelAdjustMode.DPAD;
                } else {
                    mode = flywheelAdjustMode.TRIGGER;
                }
            }
            if (gamepad1.right_trigger >= 0.01 && mode == flywheelAdjustMode.TRIGGER) {
                robot.setDesiredFlywheel(gamepad1.right_trigger);
            } else if (gamepad1.right_trigger < 0 && mode == flywheelAdjustMode.TRIGGER) {
                robot.setDesiredFlywheel(0);
            }
            if (gamepad1.dpad_down && mode == flywheelAdjustMode.DPAD) {
                flywheelSpeed -= 0.05;
            }
            if (gamepad1.dpad_up && mode == flywheelAdjustMode.DPAD) {
                flywheelSpeed += 0.05;
            }
            robot.actMotors();
            telemetry.addData("Speed", flywheelSpeed);
            telemetry.addData("Mode", mode.toString());
            telemetry.update();
            if (gamepad1.left_bumper){
                stop();
            }
        }
    }
}
