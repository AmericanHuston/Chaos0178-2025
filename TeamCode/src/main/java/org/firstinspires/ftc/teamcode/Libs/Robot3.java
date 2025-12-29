package org.firstinspires.ftc.teamcode.Libs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

public class Robot3 {
    
    double desiredFrontRight;
    double desiredFrontLeft;
    double desiredBackRight;
    double desiredBackLeft;

    double lastSuccessfulSpeed;

    private boolean isFlywheelOn = false;
    private boolean isIntakeOn = false;
    private boolean isTransferOn = false;
    private boolean isFeederLOn = false;
    private boolean isFeederROn = false;

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
    DcMotorEx FlywheelMotor;
    DcMotor IntakeMotor;
    CRServo ServoTransfer;
    CRServo FeederL;
    CRServo FeederR;
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
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        IntakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        ServoTransfer = hardwareMap.crservo.get("Mrs.Servo");
        FeederL = hardwareMap.crservo.get("FeederL");
        FeederR = hardwareMap.crservo.get("FeederR");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        IMU.initialize(parameters);
//        //Pinpoint.setOffsets(0, 155, DistanceUnit.MM);
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FeederR.setDirection(CRServo.Direction.REVERSE);
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

    public boolean getIsTransferOn(){
        return isTransferOn;
    }

    public boolean isIntakeOn() {
        return isIntakeOn;
    }

    public boolean isFeederLOn() { return isFeederLOn; }

    public boolean isFeederROn() { return isFeederROn; }

    public boolean getIsFlywheelOn(){
        return isFlywheelOn;
    }

    public double getFlywheelSpeed(){
        return FlywheelMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void setLastSuccessfulSpeed(double speed){
        this.lastSuccessfulSpeed = speed;
    }

    public double getLastSuccessfulSpeed(){
        return lastSuccessfulSpeed;
    }

    public void setFlywheelVelocity(double velocity){
        FlywheelMotor.setVelocity(velocity, AngleUnit.RADIANS);
    }

    public void stopFlywheelVelocity(){
        FlywheelMotor.setVelocity(0.0);
    }

    public double getFlywheelSpeedRPM(){
        return (this.getFlywheelSpeed() * 17.6470588);
    }

    public double getFlywheelPower(){
        return FlywheelMotor.getPower();
    }
    public void resetIMU() {
        IMU.resetYaw();
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
    public void motorTest(DcMotor motor, double power){
        motor.setPower(power);
    }
    public void intake(double power){
        IntakeMotor.setPower(power);
        isIntakeOn = !isIntakeOn;
    } //runs the intake
    public void transfer(double power){
        ServoTransfer.setPower(power);
        isTransferOn = !isTransferOn;
    }
    public void feederL(double power){
        FeederL.setPower(power);
        isFeederLOn = !isFeederLOn;
    }
    public void feederR(double power){
        FeederR.setPower(power);
        isFeederROn = !isFeederROn;
    }
    public void actMotors(){
        FrontRightMotor.setPower(desiredFrontRight);
        FrontLeftMotor.setPower(desiredFrontLeft);
        BackRightMotor.setPower(desiredBackRight);
        BackLeftMotor.setPower(desiredBackLeft);
//        Flywheel.setPower(desiredFlywheel);
    }

    public double calcPowerForFlywheel(Pose currentPosition){
        Pose GoalArea = new Pose(5, 140);

        double distanceFrom = GoalArea.distanceFrom(currentPosition);

        //Ramp Angle 120 deg
        double maxDistance = 173.07;
        double minDistance = 67.88;
        double maxPower = 70;
        double minPower =  50;

        double ratio = (maxDistance - minDistance) / (maxPower - minPower);

        double distanceFromGoal = (distanceFrom - minDistance);

        if (distanceFromGoal < 0){
            distanceFromGoal = 0;
        }

        return ( (distanceFromGoal / ratio) * 5 * 0.01 );
    }
}
