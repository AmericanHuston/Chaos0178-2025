package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Diagonal6 extends OpMode {
    Robot3 robot;
    private Follower follower;
    private Timer state_timer;
    private Timer Op_mode_timer;
    private int autoState = 0;

    public Pose StartingPose;
    public Pose ShootingPose;
    public Pose PickUpPartOne;
    public Pose PickUpPartTwo;
    public Pose EndingPose;

    private PathChain Forward;
    private PathChain Collect;
    private PathChain Park;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartingPose);
        state_timer = new Timer();
        Op_mode_timer = new Timer();
        Op_mode_timer.resetTimer();

        Forward = follower.pathBuilder()
                .addPath(new BezierLine(StartingPose, ShootingPose))
                .setLinearHeadingInterpolation(StartingPose.getHeading(), ShootingPose.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose, EndingPose))
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), EndingPose.getHeading())
                .build();
        Collect = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose, PickUpPartOne))
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), PickUpPartOne.getHeading())
                .addPath(new BezierLine(PickUpPartOne, PickUpPartTwo))
                .setLinearHeadingInterpolation(PickUpPartOne.getHeading(), PickUpPartTwo.getHeading())
                .addPath(new BezierLine(PickUpPartTwo, ShootingPose))
                .setLinearHeadingInterpolation(PickUpPartTwo.getHeading(), ShootingPose.getHeading())
                .build();

    }
    public void next_state(){
        autoState += 1;
        state_timer.resetTimer();
    }
    @Override
    public void start() {
        Op_mode_timer.resetTimer();
        autoState = 0;
        Op_mode_timer.getElapsedTimeSeconds();
    }



    @Override
    public void loop() {
        follower.update();
        robot.setLastPose(follower.getPose());
        switch (autoState){
            case 0://turns to shoot
                if (!follower.isBusy()){
                    follower.followPath(Forward);
                }
                next_state();
                break;
            case 1://shoots
                if (!follower.isBusy()){
                    robot.spinFlywheel(1140);
                    robot.transfer(1.0);
                    robot.intake(1.0);
                    if (state_timer.getElapsedTimeSeconds() > 6) {
                        robot.feederL(1.0);
                        if(state_timer.getElapsedTimeSeconds() > 8){
                            robot.feederL(0.0);
                            if (state_timer.getElapsedTimeSeconds() > 8.5) {
                                robot.feederR(1.0);
                                if (state_timer.getElapsedTimeSeconds() > 13){
                                    robot.feederR(0.0);
                                    next_state();
                                }
                            }
                        }
                    }
                }
                break;
            case 2://collect
                follower.setMaxPower(0.8);
                follower.followPath(Collect);
                next_state();
                break;
            case 3://fire
                if (!follower.isBusy()){
                    robot.transfer(1.0);
                    robot.feederR(1.0);
                    robot.feederL(1.0);
                    if (state_timer.getElapsedTimeSeconds() > 13){
                        robot.stopFlywheel();
                        robot.transfer(0.0);
                        robot.intake(0.0);
                        robot.feederL(0.0);
                        robot.feederR(0.0);
                        next_state();
                    }
                }
                break;
            case 4://park
                follower.followPath(Park);
                next_state();
                break;

        }
        robot.draw(follower);
    }
}