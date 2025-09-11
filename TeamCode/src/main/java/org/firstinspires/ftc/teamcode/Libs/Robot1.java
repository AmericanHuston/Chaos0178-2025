package org.firstinspires.ftc.teamcode.Libs;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Robot1 {


    private static Pose lastPose = new Pose(24,24, Math.toRadians(0));

    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void setLastPose(Pose savePose){
        lastPose = savePose;
    }
    public static Pose getLastPose(){
        return lastPose;
    }



    public void resetIMU() {
        imu.resetYaw();
        pinpoint.resetPosAndIMU();
    }

}