package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
@Disabled
@TeleOp(name = "TestMotors", group = "Tests")
public class TestMotors extends LinearOpMode {

    Robot3 robot = new Robot3(ConstantChaos.Alliance.RED);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.b && gamepad1.left_bumper){
                robot.setDesiredBackLeft(0.4);
                telemetry.addData("Motor", "Back Left");
            }
            if (gamepad1.b && gamepad1.right_bumper){
                robot.setDesiredBackRight(0.4);
                telemetry.addData("Motor", "Back Right");
            }
            if (gamepad1.a && gamepad1.left_bumper){
                robot.setDesiredFrontLeft(0.4);
                telemetry.addData("Motor", "Front Left");
            }
            if (gamepad1.a && gamepad1.right_bumper){
                robot.setDesiredFrontRight(0.4);
                telemetry.addData("Motor", "Front Right");
            }
            if (gamepad1.dpadDownWasPressed()){
                robot.setDesiredFrontLeft(0.0);
                robot.setDesiredFrontRight(0.0);
                robot.setDesiredBackLeft(0.0);
                robot.setDesiredBackRight(0.0);
            }
            telemetry.update();
            robot.actMotors();
        }
    }
}
