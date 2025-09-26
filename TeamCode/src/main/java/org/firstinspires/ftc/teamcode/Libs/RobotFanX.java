package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFanX {
    double desiredLeftPower = 0.0;
    double desiredRightPower = 0.0;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    public void init(HardwareMap hardwareMap) {
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
    }
    public int setDesiredPower(DcMotor motor, double power){
        if (motor.equals(backLeftMotor)) {
            desiredLeftPower = power;
            return 0;
        } else if (motor.equals(backRightMotor)) {
            desiredRightPower = power;
            return 0;
        } else {
            return 1;
        }
    }

    public void actDrive(){
        backRightMotor.setPower(desiredRightPower);
        backLeftMotor.setPower(desiredLeftPower);
    }
}
