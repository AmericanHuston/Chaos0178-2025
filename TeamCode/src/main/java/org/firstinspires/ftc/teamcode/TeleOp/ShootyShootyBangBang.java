package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import com.bylazar.panels.Panels;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

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

    Robot3 robot = new Robot3();
    Tuning Tuning = new Tuning();
    public Pose startingPose = new Pose(8,56, 0);
    public Pose fourPoint = new Pose(86, 60, 55);
    public Pose Fire1 = new Pose(72, 24);
    public double flyVel = 0.0;
    public double intakeVel = 0.0;
    public double transferVel = 0.5;
    public double flywheelPower = 0.0;

    private PathChain pointFour;
    private PathChain goToShoot;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);//new follower creator
        telemetry.addData("heading", robot.getLastPose().getHeading());
//        follower.setStartingPose(robot.getLastPose());
        follower.setStartingPose(robot.getLastPose());
        if(ConstantChaos.isRed){
            robot.GoalArea = ConstantChaos.RedGoalArea;
            Fire1 = ConstantChaos.Red1Fire;
        }else{
            robot.GoalArea = ConstantChaos.BlueGoalArea;
            Fire1 = ConstantChaos.Blue1Fire;
        }
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update(); //MUST COME BEFORE SET TELE OP DRIVE
        //Okay, if something is reversed in the driving, try swapping the polarity here
        follower.setTeleOpDrive(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        follower.updateDrivetrain();
        //Driving------------------

        if (gamepad1.left_stick_button || gamepad1.right_stick_button) { //For when we code auto points in Teleop
            follower.startTeleopDrive();
        }

        if (gamepad1.left_trigger > 0.01){ //Quarter speed
            follower.setTeleOpDrive(-gamepad1.left_stick_y/4, -gamepad1.left_stick_x/4, -gamepad1.right_stick_x/4, false);
            follower.update();
        }

        if (gamepad1.right_trigger > 0.01){ //Full speed
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();
        }
        //Driving----------------

        //Wheel tests and auto points
        if(gamepad1.aWasReleased()){
            Pose i_am_here = follower.getPose();
            telemetry.addData("i_am_here", i_am_here);
            follower.holdPoint(i_am_here);
        }
        if (gamepad1.bWasReleased()){
            robot.resetIMU();
        }
        if (gamepad1.rightBumperWasReleased()){
            Paths(follower);
            follower.followPath(goToShoot);
        }
//        if(gamepad1.b){
//            robot.setDesiredBackRight(1.0);
//            robot.actMotors();
//        }
//        if(gamepad1.y){
//            robot.setDesiredFrontLeft(1.0);
//            robot.actMotors();
//        }
//        if(gamepad1.x){
//            robot.setDesiredBackLeft(1.0);
//            robot.actMotors();
//        }
        //Wheel tests

//        if (gamepad2.a && robot.getIsFlywheelOn()){
//            robot.stopFlywheel();
//        } else if (gamepad2.a && !robot.getIsFlywheelOn()) {
//            robot.stopFlywheel();
//        }
//        if(gamepad2.dpadUpWasReleased()){
//            robot.setLastSuccessfulSpeed(robot.getFlywheelSpeed());
//        }
//        if (gamepad2.dpadLeftWasPressed()){
//            robot.stopFlywheelVelocity();
//        }

        if (gamepad2.right_trigger >= 0.01){
            robot.spinFlywheel(robot.calcPowerForFlywheel(follower.getPose()));
            flywheelPower = robot.calcPowerForFlywheel(follower.getPose());
        }else{
            robot.spinFlywheel(flyVel);
        }
//        if (gamepad2.b) {
//            List<AprilTagDetection> currentDetections = robot.getAprilTags();
//            if (currentDetections.isEmpty()) {
//                telemetry.addData("AprilTagDetections", "No tags detected");
//            } else {
//                telemetry.addData("AprilTagDetections", "Tags were detected");
//                double headingOfTagFromRobot = robot.getHeading();
//                Pose currentPose = follower.getPose();
//                Path rotationPath = new Path(new BezierLine(currentPose, currentPose.setHeading(currentPose.getHeading() + headingOfTagFromRobot)));
//                follower.followPath(rotationPath);
//            }
//        }
        if(gamepad2.rightBumperWasReleased()){ //intake on and off
            if(robot.isIntakeOn()){
                robot.intake(0.0);
            }else{
                robot.intake(1.0);
            }
        }
//        if(gamepad2.yWasReleased()){ //Servo stop
//            transferVel = 0.0;
//            robot.transfer1(transferVel);
//        }
//        if(gamepad2.xWasReleased()){ //Servo start
//            transferVel = -1.0;
//            robot.transfer1(transferVel);
//        }
//        if(gamepad2.aWasReleased()){ //Servo stop
//            transferVel = 0.0;
//            robot.transfer(transferVel);
//        }
//        if(gamepad2.bWasReleased()){ //Servo start
//            transferVel = 1.0;
//            robot.transfer(transferVel);
//        }
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

//        if (gamepad2.right_trigger >= 0.01){
//            robot.spinFlywheel(gamepad2.right_trigger);
//        }else if (gamepad2.dpadDownWasPressed()){
//            robot.setFlywheelVelocity(robot.getLastSuccessfulSpeed()); //NOTE TO SARAH/NATHAN: This should now work, please test
//        }
        if (gamepad2.dpadUpWasPressed()){
            flyVel += 0.05;
        }
        if (gamepad2.dpadDownWasPressed()){
            flyVel -= 0.05;
        }
        if (gamepad2.dpadLeftWasPressed()){
            flyVel = 0;
        }
        if (gamepad2.dpadRightWasPressed()){
            flyVel = 0.5;
        }


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
        telemetry.addData("flyVel", flyVel);
        telemetry.addData("Flywheel Speed", robot.getFlywheelSpeedRPM());
        telemetry.addData("Flywheel Power", flywheelPower);
        telemetry.addData("Transfer is On", robot.getIsTransferOn());
        telemetry.addData("Last Successful Shot Speed", robot.getLastSuccessfulSpeed());

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

        //Panels Telemetry?
        panelsTelemetry.addData("X", follower.getPose().getX());
        panelsTelemetry.addData("Y", follower.getPose().getY());
        panelsTelemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.addData("Flywheel RPM", robot.getFlywheelSpeedRPM());
        panelsTelemetry.addData("Distance to Goal", robot.DistanceFromGoal(follower.getPose()));
        panelsTelemetry.addData("CalcPowerForFlywheel", flywheelPower);
        panelsTelemetry.addData("Ticks Per Second", robot.getFlywheelVelocity());
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
                        new BezierLine(follower.getPose(), Fire1)
                )
                .setLinearHeadingInterpolation(follower.getHeading(), Fire1.getHeading())
                .build();
    }
}