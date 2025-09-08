package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="testUpload", group = "TeleOp")
public class testUpload extends OpMode {
    @Override
    public void init() {
        telemetry.addData("This ran", "yeah");
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        super.stop();
        telemetry.addData("aw man", "Gosh darn it");
    }
}
