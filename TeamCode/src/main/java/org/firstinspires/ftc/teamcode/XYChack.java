package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Chack")
public class XYChack extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.update();
        telemetry.addData("xr",gamepad1.right_stick_x);
        telemetry.addData("yr",gamepad1.right_stick_y);
        telemetry.addData("xl",gamepad1.left_stick_x);
        telemetry.addData("yl",gamepad1.left_stick_y);
        telemetry.update();
    }
}
