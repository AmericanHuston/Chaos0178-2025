package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class NoahHubRobot {
    DcMotorEx FlywheelMotor;

    double desiredFlywheelPower = 0.0;
    double desiredFlywheelVelocity = 0.0;

    public boolean isFlywheelOn = false;

    public void init(HardwareMap HwMap){
        FlywheelMotor = HwMap.get(DcMotorEx.class, "flywheel");
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
        FlywheelMotor.setVelocity(vel);
    }

    public void actMotors(){
        FlywheelMotor.setPower(desiredFlywheelPower);
    }
}
