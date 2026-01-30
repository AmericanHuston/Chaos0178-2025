package org.firstinspires.ftc.teamcode.Libs;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ConstantChaos {
    public enum Alliance {
        BLUE,
        RED
    }
    public static boolean isRed = true;

    public static double maxDistance = 144;
    public static double minDistance = 40;
    public static double maxVelocity = 1600;
    public static double minVelocity =  1000;

    // Starting Areas
    public static final Pose RedStartingPoseOffTheLine = new Pose(96,7, Math.toRadians(0));
    public static final Pose RedShootingPoseOffTheLine = new Pose(84, 14, Math.toRadians(249));
    public static final Pose RedFirstThree = new Pose(130,35, Math.toRadians(0));
    public static final Pose RedControl = new Pose(88,38);
    public static final Pose RedEndingPoseOffTheLine = new Pose(110,7, Math.toRadians(0));
    public static final Pose RedStartingPoseDiagonal = new Pose(124,124, Math.toRadians(217));
    public static final Pose RedShootingPoseDiagonal = new Pose(110, 100, Math.toRadians(217));
    public static final Pose RedEndingPoseDiagonal = new Pose(110,110, Math.toRadians(0));
    public static final Pose RedGoalArea = new Pose(130, 132);
    public static final Pose Red1Fire = new Pose(84, 93, Math.toRadians(225));

    public static final Pose BlueStartingPoseOffTheLine = new Pose(48,7, Math.toRadians(180));
    public static final Pose BlueShootingOffTheLine = new Pose(52, 14, Math.toRadians(290));
    public static final Pose BlueFirstThree = new Pose(14,36,Math.toRadians(180));
    public static final Pose BlueControl =  new Pose(50,40);
    public static final Pose BlueEndingPoseOffTheLine = new Pose(34,7, Math.toRadians(180));
    public static final Pose BlueStartingPoseDiagonal = new Pose(22, 124, Math.toRadians(323));
    public static final Pose BlueShootingPoseDiagonal = new Pose(60,100, Math.toRadians(323));
    public static final Pose BlueEndingPoseDiagonal = new Pose(60, 110, Math.toRadians(0));
    public static final Pose BlueGoalArea = new Pose(14, 132);
    public static final Pose Blue1Fire = new Pose(50, 93, Math.toRadians(225));
}
