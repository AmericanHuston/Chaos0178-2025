package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Outreach", group = "TeleOp")
public class Outreach extends OpMode {

    private Follower follower;

    Robot3 robot = new Robot3();
    public Pose startingPose = new Pose(8,56, 0);
    public Pose fourPoint = new Pose(86, 60, 55);
    public double flyVel = 0.0;
    public double intakeVel = 0.0;
    public double transferVel = 0.5;

    private PathChain pointFour;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);//new follower creator
        telemetry.addData("heading", robot.getLastPose().getHeading());
        follower.setStartingPose(robot.getLastPose());
        telemetry.addData("checking pose", robot.getLastPose());
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update(); //MUST COME BEFORE SET TELE OP DRIVE
        //Okay, if something is reversed in the driving, try swapping the polarity here
        follower.setTeleOpDrive(-gamepad1.left_stick_y/4, -gamepad1.left_stick_x/4, -gamepad1.right_stick_x/4, false);
        follower.updateDrivetrain();
        //Driving------------------

        if (gamepad1.left_stick_button || gamepad1.right_stick_button) { //For when we code auto points in Teleop
            follower.startTeleopDrive();
        }

        if (gamepad1.left_trigger > 0.01){ //Slow mode
            follower.setTeleOpDrive(-gamepad1.left_stick_y/8, -gamepad1.left_stick_x/8, -gamepad1.right_stick_x/8, false);
            follower.update();
        }

        if (gamepad1.right_trigger > 0.01){ //Full speed, for outreach
            follower.setTeleOpDrive(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
            follower.update();
        }
        //Driving----------------

        if(gamepad2.dpad_right){
            flyVel = 0.5;
            robot.spinFlywheel(flyVel);
        }
        if(gamepad2.dpad_left){
            flyVel = 0.0;
            robot.spinFlywheel(flyVel);
        }
        if(gamepad2.rightBumperWasReleased()){ //Intake on
            intakeVel = 1.0;
            robot.intake(intakeVel);
        }
        if(gamepad2.leftBumperWasReleased()){ //Intake off
            intakeVel = 0.0;
            robot.intake(intakeVel);
        }
        if(gamepad2.aWasReleased()){ //Servo stop
            transferVel = 0.0;
            robot.transfer(transferVel);
        }
        if(gamepad2.bWasReleased()){ //Servo start
            transferVel = 1.0;
            robot.transfer(transferVel);
        }

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("flyVel", flyVel);

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    @Override
    public void stop() {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}