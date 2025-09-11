package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Libs.Robot1;

@TeleOp(name = "ShootyShootyBangBang", group = "TeleOp")
public class ShootyShootyBangBang extends OpMode {

    private Follower follower;

    Robot1 robot = new Robot1();
    public Pose startingPose = new Pose(5,72, 0);

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);//new follower creator
        telemetry.addData("heading", follower.getHeading());
        follower.setStartingPose(startingPose);
        telemetry.addData("checking pose", startingPose);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        follower.updateDrivetrain();
        follower.update();
        //Driving------------------

        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            follower.startTeleopDrive();
        }
        if (gamepad1.left_trigger > 0.01){
            follower.setTeleOpDrive(-gamepad1.left_stick_y/4, -gamepad1.left_stick_x/4, -gamepad1.right_stick_x/4, false);
            follower.update();
        }
        if (gamepad1.right_trigger > 0.01){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();
        }
        //Driving----------------

        //Rewrite below----------
        if (gamepad1.back) {
            follower.setPose(new Pose(8, 71, Math.toRadians(0)));
            follower.update();
        }

        //Rewrite above----------

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    @Override
    public void stop() {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}