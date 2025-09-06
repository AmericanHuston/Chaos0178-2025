package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libs.Robot2;
@Disabled
@TeleOp(name = "AreSystemsGo", group = "SystemChecks")
public class AreSystemsGo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2 board = new Robot2();
        board.init(hardwareMap);

        DcMotor frontLeftMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backRightMotor;

        double power;

        waitForStart();

        while (opModeIsActive()){
            frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
            backLeftMotor = hardwareMap.dcMotor.get("backLeft");
            frontRightMotor = hardwareMap.dcMotor.get("frontRight");
            backRightMotor = hardwareMap.dcMotor.get("backRight");

            sleep(1000);
            //Claw
            board.openClaw();
            board.allAct();

            sleep(1000);
            //Arm
            board.setArmState(Robot2.armState.COLLECTION);
            board.allAct();

            sleep(1000);
            //Sliders
            board.setArmState(Robot2.armState.ABOVE_BAR);
            board.allAct();

            sleep(1000);
            //Wheels
            power = 0.3;
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            sleep(1000);
            power = 0.0;
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(power);
        }
    }
}