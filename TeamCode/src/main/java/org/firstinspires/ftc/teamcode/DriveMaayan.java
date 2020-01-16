package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//@Disabled
@TeleOp(name = "DriveM")
public class DriveMaayan extends OpMode {
    private DcMotor ForRight;
    private DcMotor ForLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private DcMotor LeftE;
    private DcMotor RightE;
    private Servo Griper;
    private Servo Graple;
    private Servo Servo1;
    private Servo Servo2;
    private Servo Oner1;
    private Servo Oner2;
    double i;
    double f;
    final static double SLOWING = 1.5;
    final static double POWER = 0.3;
    final static double POWER2 = 1;
    final static double EMAX = 1040;
    final static double EMIN = 50;
    @Override
    public void init() {
        ForRight = hardwareMap.get(DcMotor.class,"ForRight");
        ForLeft = hardwareMap.get(DcMotor.class,"ForLeft");
        BackRight = hardwareMap.get(DcMotor.class,"BackRight");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        RightMotor=hardwareMap.get(DcMotor.class,"RightMotor");
        LeftMotor=hardwareMap.get(DcMotor.class,"LeftMotor");
        LeftE=hardwareMap.get(DcMotor.class,"left");
        RightE=hardwareMap.get(DcMotor.class,"right");
        Griper=hardwareMap.get(Servo.class,"Griper");
        Graple=hardwareMap.get(Servo.class,"Graple");
        Servo1=hardwareMap.get(Servo.class,"Servo1");
        Servo2=hardwareMap.get(Servo.class,"Servo2");
        Oner1=hardwareMap.get(Servo.class,"Oner1");
        Oner2=hardwareMap.get(Servo.class,"Oner2");
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftE.setDirection(DcMotorSimple.Direction.REVERSE);
        Oner2.setDirection(Servo.Direction.REVERSE);
        LeftE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        telemetry.update();
        telemetry.addData("ER",RightE.getCurrentPosition());
        telemetry.addData("EL",LeftE.getCurrentPosition());
        telemetry.addData("ME",(RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2);
        if(gamepad1.dpad_left){
            i=-1;
        }else if(gamepad1.dpad_right){
            i=1;
        }if(gamepad2.dpad_up){
            f=1;
        }else if(gamepad2.dpad_down){
            f=-1;
        }
        if(gamepad1.left_bumper){
            StrafeLeft();
        }else if(gamepad1.right_bumper){
            StrafeRight();
        }else if(gamepad1.left_trigger>0.2){
            StrafeLeft2();
        }else if(gamepad1.right_trigger>0.2){
            StrafeRight2();
        }else if(gamepad1.left_bumper&&i==-1){
            StrafeLeft3();
        }else if (gamepad1.right_bumper&&i==-1){
            StrafeRight3();
        }else if (gamepad1.left_trigger>0.2&&i==-1){
            StrafeLeft4();
        }else if (gamepad1.right_trigger>0.2&&i==-1){
            StrafeRight4();
        } else if(i==-1){
            MecanumBack(gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x);
        }else{
            Mecanums(-gamepad1.left_stick_y,gamepad1.left_stick_x,-gamepad1.right_stick_x);
        }
        if (gamepad2.left_bumper){
            In();
        }else if(gamepad2.right_bumper){
            Out();
        }else{
            MotorStatic();
        }
        if(gamepad2.b||gamepad1.b){
            Up();
        }else if (gamepad2.a||gamepad1.a){
            Down();
        }else{
            ServoStatic();
        }
        if(gamepad2.y){
            GriperFor();
        }else if(gamepad2.x){
            GriperBack();
        }else {

        }
        if(gamepad2.left_stick_y<-0.2){
            Graple.setPosition(0);
            Oner2.setPosition(0);
        }else if(gamepad2.left_stick_y>0.2){
            Graple.setPosition(1);
            Oner2.setPosition(1);
        }else{
            Graple.setPosition(0.5);
            Oner2.setPosition(0.5);
        }if(f==-1 && ((RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2 >= EMIN && -gamepad2.right_stick_y<-0.111) || ((RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2 <= EMAX && -gamepad2.right_stick_y>0.222)) {
            LinearSlow(-gamepad2.right_stick_y);
        }else if(((RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2 >= EMIN && -gamepad2.right_stick_y<-0.111) || ((RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2 <= EMAX && -gamepad2.right_stick_y>0.222)){
            LinearFast(-gamepad2.right_stick_y);
        }else{
            RightE.setPower(0);
            LeftE.setPower(0);
        }
        if (gamepad2.left_trigger>0.2){
            OnerOut();
        }else if(gamepad2.right_trigger>0.2){
            OnerIn();
        }else{
            OnerStatic();
        }
        telemetry.addData("ModeMotor",i);
        telemetry.addData("ModeLinear",f);
        telemetry.addData("ForRight",ForRight.getCurrentPosition());
        telemetry.addData("ForLeft",ForLeft.getCurrentPosition());
        telemetry.addData("BackRight",BackRight.getCurrentPosition());
        telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
    }
    void In(){
        LeftMotor.setPower(1);
        RightMotor.setPower(1);
    }
    void Out(){
        LeftMotor.setPower(-1);
        RightMotor.setPower(-1);
    }
    void MotorStatic(){
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }
    void StrafeRight(){
        ForRight.setPower(POWER);
        ForLeft.setPower(POWER);
        BackRight.setPower(-POWER);
        BackLeft.setPower(-POWER);
    }
    void StrafeLeft(){
        ForRight.setPower(-POWER);
        ForLeft.setPower(-POWER);
        BackRight.setPower(POWER);
        BackLeft.setPower(POWER);
    }
    void StrafeRight2(){
        ForRight.setPower(POWER2);
        ForLeft.setPower(POWER2);
        BackRight.setPower(-POWER2);
        BackLeft.setPower(-POWER2);
    }
    void StrafeLeft2(){
        ForRight.setPower(-POWER2);
        ForLeft.setPower(-POWER2);
        BackRight.setPower(POWER2);
        BackLeft.setPower(POWER2);
    }
    void StrafeRight3(){
        ForRight.setPower(POWER);
        ForLeft.setPower(POWER);
        BackRight.setPower(-POWER);
        BackLeft.setPower(-POWER);
    }
    void StrafeLeft3(){
        ForRight.setPower(-POWER);
        ForLeft.setPower(-POWER);
        BackRight.setPower(POWER);
        BackLeft.setPower(POWER);
    }
    void StrafeRight4(){
        ForRight.setPower(-POWER2);
        ForLeft.setPower(-POWER2);
        BackRight.setPower(POWER2);
        BackLeft.setPower(POWER2);
    }
    void StrafeLeft4(){
        ForRight.setPower(POWER2);
        ForLeft.setPower(POWER2);
        BackRight.setPower(-POWER2);
        BackLeft.setPower(-POWER2);
    }
    void Mecanums(double y,double x,double spin){
        ForRight.setPower((spin-y)/SLOWING+x);
        ForLeft.setPower((spin+y)/SLOWING+x);
        BackRight.setPower((spin-y)/SLOWING-x);
        BackLeft.setPower((spin+y)/SLOWING-x);
    }
    void MecanumBack(double y,double x,double spin){
        ForRight.setPower((-spin+y)/-SLOWING-x);
        ForLeft.setPower((-spin-y)/-SLOWING-x);
        BackRight.setPower((-spin+y)/-SLOWING+x);
        BackLeft.setPower((-spin-y)/-SLOWING+x);
    }
    void Down(){
        Servo1.setPosition(0);
        Servo2.setPosition(1);
    }
    void Up(){
        Servo1.setPosition(1);
        Servo2.setPosition(0);
    }
    void ServoStatic(){
        Servo1.setPosition(0.5);
        Servo2.setPosition(0.5);
    }
    void LinearFast(double y){
        RightE.setPower(y);
        LeftE.setPower(y);
    }
    void LinearSlow(double y){
        RightE.setPower(y/2);
        LeftE.setPower(y/2);
    }
    void OnerIn(){
        Oner1.setPosition(1);
    }
    void OnerOut(){
        Oner1.setPosition(0);
    }
    void OnerStatic(){
        Oner1.setPosition(0.5);
    }
    void GriperFor(){
        Griper.setPosition(1);
    }
    void GriperBack(){
        Griper.setPosition(0);
    }
}


