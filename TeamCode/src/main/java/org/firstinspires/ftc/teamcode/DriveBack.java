package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "DriveBack",group = "Iterative Opmode")
public class DriveBack extends OpMode {
    private DcMotor ForRight;
    private DcMotor ForLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor LeftE;
    private DcMotor RightE;
    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private Servo Graple;
    private Servo Graple2;
    private Servo Griper;
    private Servo Servo1;
    private Servo Servo2;
    private Servo CapStone;
    double i;
    double d;
    final static double SLOWING = 1.5;
    final static double POWER = 0.3;
    final static double POWER2 = 1;
    final static double EMAX = 3300;
    final static double EMIN = 95;
    @Override
    public void init() {
        d = 0;
        ForRight = hardwareMap.get(DcMotor.class,"ForRight");
        ForLeft = hardwareMap.get(DcMotor.class,"ForLeft");
        BackRight = hardwareMap.get(DcMotor.class,"BackRight");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        RightMotor=hardwareMap.get(DcMotor.class,"RightMotor");
        LeftMotor=hardwareMap.get(DcMotor.class,"LeftMotor");
        LeftE=hardwareMap.get(DcMotor.class,"left");
        RightE=hardwareMap.get(DcMotor.class,"right");
        CapStone =hardwareMap.get(Servo.class,"CapStone");
        Graple=hardwareMap.get(Servo.class,"Graple");
        Graple2=hardwareMap.get(Servo.class,"Graple2");
        Griper=hardwareMap.get(Servo.class,"Griper");
        Servo1=hardwareMap.get(Servo.class,"Servo1");
        Servo2=hardwareMap.get(Servo.class,"Servo2");
        LeftE.setDirection(DcMotorSimple.Direction.REVERSE);
        Graple2.setDirection(Servo.Direction.REVERSE);
        LeftE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        if(gamepad1.dpad_left){
            i=-1;
        }else if(gamepad1.dpad_right){
            i=1;
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
        }else if (gamepad1.left_trigger>0.1&&i==-1){
            StrafeLeft4();
        }else if (gamepad1.right_trigger>0.1&&i==-1){
            StrafeRight4();
        } else if(i==-1){
            MecanumBack(-gamepad1.right_stick_y,gamepad1.right_stick_x,-gamepad1.left_stick_x);
        }else{
            Mecanums(gamepad1.right_stick_y,-gamepad1.right_stick_x,-gamepad1.left_stick_x);
        }
        if (gamepad2.left_bumper){
            In();
        }else if(gamepad2.right_bumper){
            Out();
        }else{
            MotorStatic();
        }
        if(gamepad2.a||gamepad1.b){
            Down();
        }else if (gamepad2.b||gamepad1.a){
            Up();
        }else{
            ServoStatic();
        }
        if(gamepad2.y){
            Griper.setPosition(0.7);
        }else if(gamepad2.x){
            Griper.setPosition(0.4);
        }
        if(((RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2 >= EMIN && -gamepad2.right_stick_y<-0.111) || ((RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2 <= EMAX && -gamepad2.right_stick_y>0.01)){
            LinearFast(-gamepad2.right_stick_y*2); //just for being sure
        }else{
            RightE.setPower(0);
            LeftE.setPower(0);
        }
        if (gamepad2.dpad_left){
            CupStoneBack();
        }else if(gamepad2.dpad_right){
            CupStoneOn();
        }
        if (-gamepad2.left_stick_y>0.111){
            GapleOut();
        }else if (-gamepad2.left_stick_y<-0.111){
            GrapleIn();
        }else {
            Graple.setPosition(0.5);
            Graple2.setPosition(0.5);
        }


        telemetry.update();
        telemetry.addData("ER",RightE.getCurrentPosition());
        telemetry.addData("EL",LeftE.getCurrentPosition());
        telemetry.addData("ME",(RightE.getCurrentPosition()+LeftE.getCurrentPosition())/2);
        telemetry.addData("ModeMotor",i);
        telemetry.addData("ForRight",ForRight.getCurrentPosition());
        telemetry.addData("ForLeft",ForLeft.getCurrentPosition());
        telemetry.addData("BackRight",BackRight.getCurrentPosition());
        telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
        telemetry.addData("left", Graple.getPosition());
        telemetry.addData("left", Graple2.getPosition());
    }
    void In(){
        LeftMotor.setPower(-1);
        RightMotor.setPower(1);
    }
    void Out(){
        LeftMotor.setPower(1);
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
        ForRight.setPower(spin+-y+x);
        ForLeft.setPower(spin+y+x);
        BackRight.setPower(spin+-y-x);
        BackLeft.setPower(spin+y-x);
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
    void GapleOut(){
        Graple.setPosition(0);
        Graple2.setPosition(0);
    }
    void GrapleIn(){
        Graple.setPosition(1);
        Graple2.setPosition(1);
    }
    void GriperFor(){
        Griper.setPosition(1);
    }
    void GriperBack(){
        Griper.setPosition(0);
    }
    void CupStoneOn(){
        CapStone.setPosition(1);
    }
    void CupStoneBack(){
        CapStone.setPosition(0);
    }
}

