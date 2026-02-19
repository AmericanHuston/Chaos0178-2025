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
    public static double maxVelocity = 1650;
    public static double minVelocity =  1100;

    public static double flyVel = 1575;

    //reset Pose
    public static final Pose resetPose = new Pose(72, 9, Math.toRadians(90));

    // Starting Areas
    public static final Pose RedStartingPoseOffTheLine = new Pose(96,7, Math.toRadians(0));
    public static final Pose RedShootingPoseOffTheLine = new Pose(84, 14, Math.toRadians(245));
    public static final Pose RedFirstThree = new Pose(130,36, Math.toRadians(0));
    public static final Pose RedFirstThreePartOne = new Pose(103, 36, Math.toRadians(0));
    public static final Pose RedEndingPoseOffTheLine = new Pose(110,10, Math.toRadians(0));
    public static final Pose RedStartingPoseDiagonal = new Pose(126,120, Math.toRadians(216));
    public static final Pose RedShootingPoseDiagonal = new Pose(100, 100, Math.toRadians(217));
    public static final Pose RedPickUpPartOne = new Pose(100, 90, Math.toRadians(0));
    public static final Pose RedPickUpPartTwo = new Pose(130, 80, Math.toRadians(0));
    public static final Pose RedEndingPoseDiagonal = new Pose(110,110, Math.toRadians(0));
    public static final Pose RedGoalArea = new Pose(130, 132);
    public static final Pose RedParkArea = new Pose(38,33);
    public static final Pose Red1Fire = new Pose(84, 93, Math.toRadians(225));

    public static final Pose BlueStartingPoseOffTheLine = new Pose(48,7, Math.toRadians(180));
    public static final Pose BlueShootingOffTheLine = new Pose(52, 14, Math.toRadians(285));
    public static final Pose BlueFirstThreePartOne = new Pose(42, 36, Math.toRadians(180));
    public static final Pose BlueFirstThree = new Pose(14,36,Math.toRadians(180));
    public static final Pose BlueEndingPoseOffTheLine = new Pose(34,10, Math.toRadians(180));
    public static final Pose BlueStartingPoseDiagonal = new Pose(18, 120, Math.toRadians(324));
    public static final Pose BlueShootingPoseDiagonal = new Pose(45,100, Math.toRadians(323));
    public static final Pose BluePickUpPartOne = new Pose(50, 90, Math.toRadians(180));
    public static final Pose BluePickUpPartTwo = new Pose(14, 80, Math.toRadians(180));
    public static final Pose BlueEndingPoseDiagonal = new Pose(60, 110, Math.toRadians(0));
    public static final Pose BlueGoalArea = new Pose(14, 132);
    public static final Pose BlueParkArea = new Pose(105,33);
    public static final Pose Blue1Fire = new Pose(50, 93, Math.toRadians(225));
}
