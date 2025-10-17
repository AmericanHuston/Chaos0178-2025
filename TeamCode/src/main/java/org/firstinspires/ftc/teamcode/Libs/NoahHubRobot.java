package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class NoahHubRobot {
    DcMotorEx FlywheelMotor;
    LED GLED;
    LED RLED;

    private double desiredFlywheelPower = 0.0;
    private double desiredFlywheelVelocity = 0.0;

    public boolean isFlywheelOn = false;

    public void init(HardwareMap HwMap){
        FlywheelMotor = HwMap.get(DcMotorEx.class, "flywheel");
        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        GLED = HwMap.get(LED.class, "greenled");
        RLED = HwMap.get(LED.class, "redled");
        RLED.enable(true);
        GLED.enable(true);
    }

    public void setGreenLED(boolean light_on) {
        if (light_on) {
            GLED.on();
        } else {
            GLED.off();
        }
    }

    public void setRedLED(boolean light_on) {
        if (light_on) {
            RLED.on();
        } else {
            RLED.off();
        }
    }
    public void setAllLEDOff(){
        setRedLED(false);
        setGreenLED(false);
    }
    public boolean isFlywheelOn() {
        return isFlywheelOn;
    }

    public void setDesiredFlywheel(double power) {
        desiredFlywheelPower = power;
    }

    public double getFlywheelVelocity(){
        return FlywheelMotor.getVelocity(AngleUnit.DEGREES);
    }

    public double getFlywheelActualPower(){
        return FlywheelMotor.getPower();
    }

    public void setFlywheelVelocity(double vel){
        desiredFlywheelVelocity = vel;
    }

    public void actMotors(){
        FlywheelMotor.setPower(desiredFlywheelPower);
    }

    public void actMorosVel(){
        FlywheelMotor.setVelocity(desiredFlywheelVelocity);
    }
}
