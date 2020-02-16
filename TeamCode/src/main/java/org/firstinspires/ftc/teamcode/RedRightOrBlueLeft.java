package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedRight,BlueLeft")
@Disabled
public class RedRightOrBlueLeft extends LinearOpMode {

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
    private Servo CupStone;
    static final double COUNTS_PER_MOTOR_REV = 2240;
    static final double DRIVE_GEAR_REDUCTION = 15;
    static final double WHEEL_DIAMETER_MM = 75;
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        CupStone = hardwareMap.get(Servo.class, "CupStone");
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
        Servo2.setDirection(Servo.Direction.REVERSE);
        LeftE.setDirection(DcMotorSimple.Direction.REVERSE);
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
        LeftE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        waitForStart();
        EncodersControl(1,-600,600,600,-600,0,0,0,0);
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    public void EncodersControl ( double speed, int leftForMM, int rightForMM, int leftBackMM,int rightBackMM, double rightMotorPower, double leftMotorPower, int RightEMM, int LeftEMM){
        int newTargetForRight;
        int newTargetForLeft;
        int newTargetBackRight;
        int newTargetBackLeft;
        int newTargetRightE;
        int newTargetLeftE;
        double newTargetRightMotor;
        double newTargetLeftMotor;

        if (opModeIsActive()) {

            newTargetForRight = ForRight.getCurrentPosition() + rightForMM;
            newTargetForLeft = ForLeft.getCurrentPosition() + leftForMM;
            newTargetBackRight = BackRight.getCurrentPosition() + rightBackMM;
            newTargetBackLeft = BackLeft.getCurrentPosition() + leftBackMM;
            newTargetRightE = RightE.getCurrentPosition() + RightEMM;
            newTargetLeftE = LeftE.getCurrentPosition() + LeftEMM;
            newTargetRightMotor = rightMotorPower;
            newTargetLeftMotor = leftMotorPower;


            ForRight.setTargetPosition(newTargetForRight);
            ForLeft.setTargetPosition(newTargetForLeft);
            BackRight.setTargetPosition(newTargetBackRight);
            BackLeft.setTargetPosition(newTargetBackLeft);
            RightE.setTargetPosition(newTargetRightE);
            LeftE.setTargetPosition(newTargetLeftE);
            RightMotor.setPower(newTargetRightMotor);
            LeftMotor.setPower(newTargetLeftMotor);

            ForRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ForLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            ForRight.setPower(Math.abs(speed));
            ForLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            RightE.setPower(Math.abs(speed));
            LeftE.setPower(Math.abs(speed));

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
            RightE.setPower(0);
            LeftE.setPower(0);

            ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
