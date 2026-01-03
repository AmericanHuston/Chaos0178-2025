package org.firstinspires.ftc.teamcode.Libs;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ConstantChaos {
    public static double maxDistance = 173.07;
    public static double minDistance = 67.88;
    public static double maxVelocity = 4000;
    public static double minVelocity =  50;

    public static Pose RedGoalArea = new Pose(130, 140);
    public static Pose BlueGoalArea = new Pose(5, 140);
    //This is the first firing position for red
    public static Pose Red1Fire = new Pose(84, 93, Math.toRadians(225));

    //This is the first firing position for blue
    public static Pose Blue1Fire = new Pose(36, 93, Math.toRadians(225));

    public static boolean isRed = true;
}
