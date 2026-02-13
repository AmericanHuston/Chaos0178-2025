package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(group = "Tests", name = "Tune Flywheel PIDF")
public class TuneFlywheel extends OpMode {
    double P = 45.0;
    double F = 16.0;
    DcMotorEx FlywheelMotor;
    double highVel = 1500;
    double lowVel = 900;
    double curVel = 0.0;
    double curTargetVel = highVel;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()){
            if (curTargetVel == highVel){
                curTargetVel = lowVel;
            }else{
                curTargetVel = highVel;
            }
        }

        if (gamepad1.rightBumperWasPressed()){
            curTargetVel += stepSizes[stepIndex] * 10;
        }

        if (gamepad1.leftBumperWasPressed()){
            curTargetVel -= stepSizes[stepIndex] * 10;
        }

        if (gamepad1.bWasReleased()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        FlywheelMotor.setVelocity(curTargetVel);

        double curVelocity = FlywheelMotor.getVelocity();
        double error = curTargetVel - curVelocity;

        telemetry.addData("Power", "%.2f", FlywheelMotor.getPower());
        telemetry.addData("Target Velocity", curTargetVel);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------------");
        telemetry.addData("Tuning P", "%.4f (DPad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (DPad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
