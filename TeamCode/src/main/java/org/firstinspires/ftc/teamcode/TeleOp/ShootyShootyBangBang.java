package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ShootyShootyBangBang", group = "TeleOp")
public class ShootyShootyBangBang extends OpMode {

    private Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    Robot3 robot = new Robot3(ConstantChaos.isRed);

    public double intakeVel = 0.0;
    public double transferVel = 0.5;
    public double desiredFlywheelVelocity = 0.0;

    private PathChain goToShoot;
    private PathChain turnToShoot;
    private PathChain Park;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);//new follower creator
        telemetry.addData("heading", robot.getLastPose().getHeading());
        follower.setStartingPose(robot.getLastPose());
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update(); //MUST COME BEFORE SET TELE OP DRIVE
        //Okay, if something is reversed in the driving, try swapping the polarity here
        if (ConstantChaos.isRed) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        } else {
            follower.setTeleOpDrive(gamepad1.left_stick_y/2, gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        }
        follower.updateDrivetrain();
        //Driving------------------

        if (gamepad1.left_stick_button || gamepad1.right_stick_button) { //For when we code auto points in Teleop
            follower.startTeleopDrive();
        }

        if (gamepad1.left_trigger > 0.01){ //Quarter speed
            if (ConstantChaos.isRed) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y/4, -gamepad1.left_stick_x/4, -gamepad1.right_stick_x/4, false);
            } else {
                follower.setTeleOpDrive(gamepad1.left_stick_y/4, gamepad1.left_stick_x/4, -gamepad1.right_stick_x/4, false);
            }
            follower.update();
        }

        if (gamepad1.right_trigger > 0.01){ //Full speed
            if (ConstantChaos.isRed) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            } else {
                follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            }
            follower.update();
        }
        //Driving----------------

        //Auto points
        if(gamepad1.aWasReleased()){
            Paths(follower);
            follower.followPath(turnToShoot);
        }

        if (gamepad1.bWasReleased()){
            follower.holdPoint(follower.getPose());
        }
        if (gamepad1.rightBumperWasReleased()){
            Paths(follower);
            follower.followPath(goToShoot);
        }
        if (gamepad1.yWasReleased()){
            Paths(follower);
            follower.followPath(Park);
        }

        desiredFlywheelVelocity = robot.calcPowerForFlywheel(follower.getPose());
        if (gamepad2.right_trigger >= 0.01){
            if (follower.getPose().getY() > 50) {
                robot.spinFlywheel(desiredFlywheelVelocity);
            }else{
                robot.spinFlywheel(ConstantChaos.flyVel);
            }
        }
        if(gamepad2.leftBumperWasReleased()){ //intake on and off
            if(robot.isIntakeOn()){
                robot.intake(0.0);
            }else{
                robot.intake(1.0);
            }
        }
        if (gamepad2.aWasReleased()){ //This turns the transfer on/off
            if (robot.getIsTransferOn()){
                robot.transfer(0.0);
            }else{
                robot.transfer(1.0);
            }
        }
        if (gamepad2.xWasReleased()){ //This turns the right feeder on/off
            if(robot.isFeederROn()){
                robot.feederR(0.0);
            }else{
                robot.feederR(1.0);
            }
        }
        if (gamepad2.bWasReleased()){ //This turns the left feeder on/off
            if(robot.isFeederLOn()){
                robot.feederL(0.0);
            }else{
                robot.feederL(1.0);
            }
        }




        //Rewrite below----------
        if (gamepad1.back) {
            follower.setPose(new Pose(8, 71, Math.toRadians(0)));
            follower.update();
        }

        //Rewrite above----------

        //Panels Telemetry?
        panelsTelemetry.addData("Current Position:", String.format("(X:%.2f, Y:%.2f, θ:%.2f)", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.addData("Flywheel RPM", robot.getFlywheelSpeedRPM());
        panelsTelemetry.addData("Distance to Goal", String.format("%.2f", robot.DistanceFromGoal(follower.getPose())));
        panelsTelemetry.addData("CalcPowerForFlywheel", String.format("%.2f", desiredFlywheelVelocity));
        panelsTelemetry.addData("CalcHeadingToGoal", String.format("%.2f", Math.toDegrees(robot.calcHeadingToGoal(follower.getPose()))));
        panelsTelemetry.addData("Ticks Per Second", robot.getFlywheelVelocity());
        panelsTelemetry.addData("flyVel", ConstantChaos.flyVel);
        panelsTelemetry.addData("Transfer is On", robot.getIsTransferOn());
        panelsTelemetry.addData("Last Successful Shot Speed", robot.getLastSuccessfulSpeed());
        panelsTelemetry.update(telemetry);

        robot.draw(follower);

    }
    @Override
    public void stop() {
        telemetry.addLine("Stopped");
        telemetry.update();
    }

    public void Paths(Follower follower) {
        goToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), robot.Fire1)
                )
                .setLinearHeadingInterpolation(follower.getHeading(), robot.Fire1.getHeading())
                .build();
        turnToShoot = follower
                .pathBuilder()
                .addPath(new BezierLine(follower.getPose(), robot.getTurn(follower.getPose())))
                .setLinearHeadingInterpolation(follower.getHeading(), robot.calcHeadingToGoal(follower.getPose()))
                .build();
        Park = follower
                .pathBuilder()
                .addPath(new BezierLine(follower.getPose(), robot.Park))
                .setLinearHeadingInterpolation(follower.getHeading(), robot.Park.getHeading())
                .build();
    }
}