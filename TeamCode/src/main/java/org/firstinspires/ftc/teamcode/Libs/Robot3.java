package org.firstinspires.ftc.teamcode.Libs;
import org.firstinspires.ftc.teamcode.Libs.ConstantChaos.Alliance;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
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

    private final Alliance myAlliance;
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

    public Pose GoalArea = new Pose(72, 72);
    public Pose Fire1 = new Pose(72, 24);


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

    public Robot3(Alliance alliance) {
        // Do good things with the alliance color
        myAlliance = alliance;
        if (alliance == Alliance.RED) {
            GoalArea = ConstantChaos.RedGoalArea;
            Fire1 = ConstantChaos.Red1Fire;
        } else {
            GoalArea = ConstantChaos.BlueGoalArea;
            Fire1 = ConstantChaos.Blue1Fire;
        }
    }

    public void init(HardwareMap hardwareMap) {
        Drawing.init();
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
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FeederR.setDirection(CRServo.Direction.REVERSE);
        AprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), AprilTag);
    }

    public double getHeading(){
        return getHeadingOfTag.getHeading(visionPortal, AprilTag);
    }

    public void setLastPose(Pose savePose){
        lastPose = savePose;
    }

    public Pose getLastPose(){
        return lastPose;
    }

    public boolean getIsTransferOn(){
        return isTransferOn;
    }

    public boolean isIntakeOn() {
        return isIntakeOn;
    }

    public Alliance getAlliance() { return myAlliance; }

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

    public double getFlywheelVelocity(){ return FlywheelMotor.getVelocity(); }

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
        FlywheelMotor.setVelocity(power);
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

    public double calcHeadingToGoal(Pose currentPosition) {
        return Math.atan2(GoalArea.getY() - currentPosition.getY(), GoalArea.getX() - currentPosition.getX());
    }

    public double calcPowerForFlywheel(Pose currentPosition){
        double slope = (ConstantChaos.maxVelocity - ConstantChaos.minVelocity) / (ConstantChaos.maxDistance - ConstantChaos.minDistance);
        return slope * (DistanceFromGoal(currentPosition) - ConstantChaos.minDistance) + ConstantChaos.minVelocity;
    }

    public double DistanceFromGoal(Pose current){
        return GoalArea.distanceFrom(current);
    }
    public static void drawOnlyCurrent(Pose current) {
        try {
            Drawing.drawRobot(current);
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public void draw(Follower follower) {
        Drawing.drawDebug(follower);
    }


}


class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
