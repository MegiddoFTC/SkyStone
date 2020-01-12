package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name = "JackBlueGlow")
public class JackBlueGlow extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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
    private Servo Oner;
    static final double COUNTS_PER_MOTOR_REV = 2240;
    static final double DRIVE_GEAR_REDUCTION = 15;
    static final double WHEEL_DIAMETER_MM = 75;
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        ForRight = hardwareMap.get(DcMotor.class, "ForRight");
        ForLeft = hardwareMap.get(DcMotor.class, "ForLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
        LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
        LeftE = hardwareMap.get(DcMotor.class, "left");
        RightE = hardwareMap.get(DcMotor.class, "right");
        Griper = hardwareMap.get(Servo.class, "Griper");
        Graple = hardwareMap.get(Servo.class, "Graple");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Oner = hardwareMap.get(Servo.class, "Oner");
        Servo1.setDirection(Servo.Direction.REVERSE);
        ForLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ForLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        ForRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        waitForStart();

        EncodersControl(0.8, 1150, 1150, 1150, 1150, -0.4, -0.4, 0.5);
        EncodersControl(0.3, 800, 800, 800, 800, -1, -1, 1);
        EncodersControl(1, -1200, -1200, -1200, -1200, -1, -1, 1);
        EncodersControl(1, -3500, 3500, 3500, -3500, -1, -1, 1);
        Griper.setPosition(0);
        EncodersControl(1, -1800, 1800, -1800, 1800, 0, 0, 1);
        EncodersControl(0.3, -500, -500, -500, -500, 0, 0, 0.5);
        Graple.setPosition(0);
        Servo1.setPosition(0.7);
        Servo2.setPosition(0.7);
        sleep(3000);
        Griper.setPosition(0.6);
        sleep(1000);
        Graple.setPosition(1);
        sleep(1000);
        EncodersControl(0.6, 1350, 1350, 1350, 1350, 0, 0, 0.5);
        EncodersControl(0.4, -2200, 2200, -2200, 2200, 0, 0, 0.5);
        EncodersControl(0.6, -1300, -1300, -1300, -1300, 0, 0, 0.5);
        Servo1.setPosition(0);
        Servo2.setPosition(0);
        sleep(500);
        EncodersControl(0.6, -300, 300, 300, -300, 0, 0, 0.5);
        EncodersControl(0.6, 3000, 3000, 3000, 3000, 0, 0, 0.5);
        EncodersControl(0.6, -1000, 1000, 1000, -1000, 0, 0, 0.5);
        Griper.setPosition(0.6);
        EncodersControl(0.4, 700, 700, 700, 700, -1, -1, 1);
        EncodersControl(0.6, 700, -700, -700, 700, -1, -1, 1);
        Griper.setPosition(0);
        EncodersControl(0.6, -3700, -3700, -3700, -3700, 0, 0, 0.5);
        Graple.setPosition(0);
        sleep(3000);
        Griper.setPosition(0.6);
        sleep(1000);
        Griper.setPosition(0);
        Graple.setPosition(1);
        EncodersControl(1, 1800, 1800, 1800, 1800, 0, 0, 0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void EncodersControl(double speed, int leftForMM, int rightForMM, int leftBackMM, int rightBackMM, double rightMotorPower, double leftMotorPower, double ServoPower) {
        int newTargetForRight;
        int newTargetForLeft;
        int newTargetBackRight;
        int newTargetBackLeft;
        double newTargetRightMotor;
        double newTargetLeftMotor;

        if (opModeIsActive()) {

            newTargetForRight = ForRight.getCurrentPosition() + rightForMM;
            newTargetForLeft = ForLeft.getCurrentPosition() + leftForMM;
            newTargetBackRight = BackRight.getCurrentPosition() + rightBackMM;
            newTargetBackLeft = BackLeft.getCurrentPosition() + leftBackMM;
            newTargetRightMotor = rightMotorPower;
            newTargetLeftMotor = leftMotorPower;


            ForRight.setTargetPosition(newTargetForRight);
            ForLeft.setTargetPosition(newTargetForLeft);
            BackRight.setTargetPosition(newTargetBackRight);
            BackLeft.setTargetPosition(newTargetBackLeft);
            RightMotor.setPower(newTargetRightMotor);
            LeftMotor.setPower(newTargetLeftMotor);
            Oner.setPosition(ServoPower);

            ForRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ForLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            ForRight.setPower(Math.abs(speed));
            ForLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));

            while (opModeIsActive() && (ForRight.isBusy() && ForLeft.isBusy() && BackRight.isBusy() && BackLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newTargetForRight, newTargetForLeft, newTargetBackRight, newTargetBackLeft);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        ForRight.getCurrentPosition(),
                        ForLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition(),
                        BackLeft.getCurrentPosition());
                telemetry.update();
            }

            ForRight.setPower(0);
            ForLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
            RightMotor.setPower(0);
            LeftMotor.setPower(0);


            ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}