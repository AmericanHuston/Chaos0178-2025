package org.firstinspires.ftc.teamcode.Libs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.firstinspires.ftc.teamcode.Libs.getHeadingOfTag;

import java.util.List;

public class Robot3 {
    
    double desiredFrontRight;
    double desiredFrontLeft;
    double desiredBackRight;
    double desiredBackLeft;

    private boolean isFlywheelOn = false;

    private static Pose lastPose = new Pose(24,24, Math.toRadians(0));
    private static Pose startingPose1 = new Pose(0,0,0); //TODO Populate Data
    private static Pose startingPose2 = new Pose(0,0,0); //TODO Populate Data
    private static Pose startingPose3 = new Pose(0,0,0); //TODO Populate Data
    private static Pose startingPose4 = new Pose(0,0,0); //TODO Populate Data

    IMU IMU;
    DcMotor FrontLeftMotor;
    DcMotor BackLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackRightMotor;
    DcMotor FlywheelMotor;
    DcMotor IntakeMotor;
    CRServo TransferServo;
    //GoBildaPinpointDriver Pinpoint;
    AprilTagProcessor AprilTag;
    VisionPortal visionPortal;

    public void init(HardwareMap hardwareMap) {
        //Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
            .setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE)
            .build();

        IMU = hardwareMap.get(IMU.class, "imu");
        FrontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        BackLeftMotor = hardwareMap.dcMotor.get("backLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("frontRight");
        BackRightMotor = hardwareMap.dcMotor.get("backRight");
        FlywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
        IntakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        TransferServo = hardwareMap.crservo.get("Mr.Servo");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        IMU.initialize(parameters);
//        //Pinpoint.setOffsets(0, 155, DistanceUnit.MM);
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), AprilTag);
    }

    public double getHeading(){
        return getHeadingOfTag.getHeading(visionPortal, AprilTag);
    }

    public static void setLastPose(Pose savePose){
        lastPose = savePose;
    }

    public static Pose getLastPose(){
        return lastPose;
    }

    public boolean getIsFlywheelOn(){
        return isFlywheelOn;
    }

    public void resetIMU() {
        IMU.resetYaw();
        //Pinpoint.resetPosAndIMU();
    }

    public List<AprilTagDetection> getAprilTags(){
        return AprilTag.getDetections();
    }

    public void setDesiredBackLeft(double desiredBackLeft) {
        this.desiredBackLeft = desiredBackLeft;
    }

    public void setDesiredBackRight(double desiredBackRight) {
        this.desiredBackRight = desiredBackRight;
    }

    public void setDesiredFrontLeft(double desiredFrontLeft) {
        this.desiredFrontLeft = desiredFrontLeft;
    }

    public void setDesiredFrontRight(double desiredFrontRight) {
        this.desiredFrontRight = desiredFrontRight;
    }

    public void spinFlywheel(double power){
        FlywheelMotor.setPower(power);
        isFlywheelOn = true;
    }

    public void stopFlywheel(){
        FlywheelMotor.setPower(0.0);
        isFlywheelOn = false;
    }
    public void intake(double power){
        IntakeMotor.setPower(power);
    }
    public void transfer(double power){
        TransferServo.setPower(power);
    }
    public void actMotors(){
        FrontRightMotor.setPower(desiredFrontRight);
        FrontLeftMotor.setPower(desiredFrontLeft);
        BackRightMotor.setPower(desiredBackRight);
        BackLeftMotor.setPower(desiredBackLeft);
//        Flywheel.setPower(desiredFlywheel);
    }
}
