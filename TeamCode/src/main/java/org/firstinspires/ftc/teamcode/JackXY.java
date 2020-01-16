package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JackXY")
public class JackXY extends OpMode {
private DcMotor ForRight;
private DcMotor ForLeft;
private DcMotor BackRight;
private DcMotor BackLeft;
    final static double SLOWING = 1.5;
double x;
double y;
double sp;
double newX;
double newY;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void init() {
        ForRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        ForRight=hardwareMap.get(DcMotor.class,"ForRight");
        ForLeft=hardwareMap.get(DcMotor.class,"ForLeft");
        BackRight=hardwareMap.get(DcMotor.class,"BackRight");
        BackLeft=hardwareMap.get(DcMotor.class,"BackLeft");
    }

    @Override
    public void loop() {
    x=gamepad1.left_stick_x;
    y=gamepad1.left_stick_y;
    sp=gamepad1.right_stick_x;

//        (Math.sqrt(x*x+y*y)*Math.sin(Math.atan(y/x)))=newY;
//        (Math.sqrt(x*x+y*y)*Math.sin(Math.atan(y/x)))=newX;

        Mecanums(newY,newX,sp);

    }
    void Mecanums(double y,double x,double spin){
        ForRight.setPower((spin-y)/SLOWING+x);
        ForLeft.setPower((spin+y)/SLOWING+x);
        BackRight.setPower((spin-y)/SLOWING-x);
        BackLeft.setPower((spin+y)/SLOWING-x);
    }

}
