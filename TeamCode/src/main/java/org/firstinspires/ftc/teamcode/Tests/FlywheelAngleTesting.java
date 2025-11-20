package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.NoahHubRobot;
@Disabled
@TeleOp(name = "FlywheelAngleTesting", group = "tests")
public class FlywheelAngleTesting extends LinearOpMode {
    enum flywheelAdjustMode {
        DPAD,
        TRIGGER
    }

    double increments = 0.01;
    flywheelAdjustMode mode = flywheelAdjustMode.DPAD;
    double flywheelPower = 0.00;
    NoahHubRobot robot = new NoahHubRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.setAllLEDOff();
        robot.setGreenLED(true);
        while (!gamepad1.aWasReleased()){
            telemetry.addData("Press", "A to Start");
            telemetry.update();
        }
        robot.setAllLEDOff();
        robot.setGreenLED(true);
        while (opModeIsActive()) {
            if (gamepad1.aWasReleased()) {
                if (mode == flywheelAdjustMode.TRIGGER) {
                    mode = flywheelAdjustMode.DPAD;
                } else {
                    mode = flywheelAdjustMode.TRIGGER;
                }
            }
            if (gamepad1.right_trigger >= 0.01 && mode == flywheelAdjustMode.TRIGGER) {
                flywheelPower = gamepad1.right_trigger;
            } else if (gamepad1.right_trigger < 0 && mode == flywheelAdjustMode.TRIGGER) {
                flywheelPower = 0;
            }
            if (gamepad1.dpadDownWasReleased() && mode == flywheelAdjustMode.DPAD) {
                flywheelPower -= increments;
            }
            if (gamepad1.dpadUpWasReleased() && mode == flywheelAdjustMode.DPAD) {
                flywheelPower += increments;
            }
            if (gamepad1.xWasReleased()) {
                flywheelPower = 0.0;
            }
            if (gamepad1.leftBumperWasReleased()){
                increments += 0.01;
            }
            if (gamepad1.rightBumperWasReleased()){
                increments -= 0.01;
            }
            while (gamepad1.b){
                robot.setFlywheelVelocity(gamepad1.left_trigger * 6000);
                telemetry.addData("Power", robot.getFlywheelActualPower());
                telemetry.addData("Speed", (robot.getFlywheelVelocity() * 17.6470588) + " RPM (Approx)"); //Approx RPM
                telemetry.addData("Mode", "VELOCITY");
                telemetry.update();
            }
            if (robot.getFlywheelVelocity() * 17.6470588 > 5000) {
                robot.setGreenLED(true);
                robot.setRedLED(false);
                telemetry.addData("LED", "GREEN");
            } else {
                robot.setRedLED(true);
                robot.setGreenLED(false);
                telemetry.addData("LED", "RED");
            }
            robot.setDesiredFlywheel(flywheelPower);
            robot.actMotors();
            telemetry.addData("Power", flywheelPower);
            telemetry.addData("Speed", (robot.getFlywheelVelocity() * 17.6470588) + " RPM (Approx)"); //Approx RPM
            telemetry.addData("Mode", mode.toString());
            telemetry.addData("Increments", increments);
            telemetry.update();
            if (isStopRequested()){
                robot.setAllLEDOff();
                stop();
            }
        }
    }
}
