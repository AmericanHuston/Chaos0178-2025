package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libs.ConstantChaos;
import org.firstinspires.ftc.teamcode.Libs.Robot3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red3Diagonal", group = "Tests")
public class Red3Diagonal extends OpMode {
    Robot3 robot = new Robot3(ConstantChaos.Alliance.BLUE);
    private Follower follower;
    private Timer state_timer;
    private Timer Op_mode_timer;
    private int autoState = 0;

    private final Pose StartingPose = ConstantChaos.RedStartingPoseDiagonal;
    private final Pose ShootingPose = ConstantChaos.RedShootingPoseDiagonal;
    private final Pose EndingPose = ConstantChaos.RedEndingPoseDiagonal;

    boolean isRed = ConstantChaos.isRed = true;//This is very important do not mix up or remove

    private PathChain Forward;
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
                    robot.spinFlywheel(1120);
                    robot.transfer(1.0);
                    robot.intake(1.0);
                    if (state_timer.getElapsedTimeSeconds() > 1) {
                        robot.feederR(1.0);
                        if(state_timer.getElapsedTimeSeconds() > 6){
                            robot.feederR(0.0);
                            robot.spinFlywheel(1160);
                            if (state_timer.getElapsedTimeSeconds() > 6.5) {
                                robot.feederL(1.0);
                                if (state_timer.getElapsedTimeSeconds() > 9){
                                    robot.feederL(0.0);
                                    if(state_timer.getElapsedTimeSeconds() > 10.5){
                                        robot.feederL(1.0);
                                        if (state_timer.getElapsedTimeSeconds() > 16){
                                            robot.stopFlywheel();
                                            robot.intake(0.0);
                                            robot.feederR(0.0);
                                            robot.transfer(0.0);
                                            next_state();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            case 2://park
                follower.followPath(Park);
                next_state();
                break;
        }
        robot.draw(follower);
    }
}