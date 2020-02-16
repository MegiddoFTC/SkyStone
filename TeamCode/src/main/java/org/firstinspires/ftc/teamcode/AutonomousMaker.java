package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;

//@Disabled
@Autonomous(name = "AutonomousMaker")
public class AutonomousMaker extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    double gg;
    double numnum;
    private DcMotorEx ForRight;
    private DcMotorEx ForLeft;
    private DcMotorEx BackRight;
    private DcMotorEx BackLeft;
    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private DcMotor LeftE;
    private DcMotor RightE;
    private Servo Griper;
    private Servo Graple;
    private Servo Servo1;
    private Servo Servo2;
    private Servo CupStone;
    OpenCvCamera phoneCam;
    static final double COUNTS_PER_MOTOR_REV = 2240;
    static final double WHEEL_DIAMETER_MM = 75;
    static final double DRIVE_GEAR_REDUCTION = 15;
    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.15;

        @Override
        public void runOpMode() {
            ForRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "ForRight");
            ForLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "ForLeft");
            BackRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "BackRight");
            BackLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "BackLeft");
            RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
            LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
            LeftE = hardwareMap.get(DcMotor.class, "left");
            RightE = hardwareMap.get(DcMotor.class, "right");
            CupStone = hardwareMap.get(Servo.class,"CapStone");
            Griper = hardwareMap.get(Servo.class, "Griper");
            Graple = hardwareMap.get(Servo.class, "Graple");
            Servo1 = hardwareMap.get(Servo.class, "Servo1");
            Servo2 = hardwareMap.get(Servo.class, "Servo2");
            Servo2.setDirection(Servo.Direction.REVERSE);
            LeftE.setDirection(DcMotorSimple.Direction.REVERSE);
            ForLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            RightE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ForRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ForLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            composeTelemetry();
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCam.openCameraDevice();
            phoneCam.setPipeline(new SamplePipeline());
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            waitForStart();

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


            while (opModeIsActive()) {
                telemetry.addData("Runtime", "%.03f", getRuntime());
                telemetry.update();
                telemetry.addData("gg value",gg);
                telemetry.addData("Frame Count", phoneCam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
                telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
                telemetry.update();
                if (gg == 69){
                    gg=69;
                    phoneCam.stopStreaming();
                    break;
                } else if (gg == 420){
                    gg=420;
                    phoneCam.stopStreaming();
                    break;
                }else  if (gg == 254){
                    gg=254;
                    phoneCam.stopStreaming();

                    break;
                }
            }
           if (gg==420){
               EncodersControl(0.6,1150,1150,1150,1150,1,1,0,0);
               EncodersControl(0.3,900,900,900,900,1,1,0,0);
               EncodersControl(0.6,-950,-950,-950,-950,1,1,0,0);
               EncodersControl(1,915,-915,915,-915,1,1,0,0);
               gyroTurn(0.2,273);
               EncodersControl(0.6,-3350,-3350,-3350,-3350,0,0,0,0);
               EncodersControl(1,915,-915,915,-915,1,1,0,0);
               gyroTurn(0.2,182);
               EncodersControl(0.3,-500,-500,-500,-500,0,0,0,0);
               Servo1.setPosition(0.8);
               Servo2.setPosition(0.8);
               gyroHold(0.1,182,2);
               Servo1.setPosition(0.5);
               Servo2.setPosition(0.5);
               EncodersControl(0.8,900,900,900,900,0,0,0,0);
               Servo1.setPosition(0.6);
               Servo2.setPosition(0.6);
               EncodersControl(0.8,-1800,1800,-1800,1800,0,0,0,0);
               Servo1.setPosition(0.3);
               Servo2.setPosition(0.3);
               EncodersControl(1,-1000,-1000,-1000,-1000,0,0,0,0);
               EncodersControl(0.8,1600,1600,1600,1600,0,0,0,0);
               EncodersControl(0.8,-400,400,400,-400,0,0,0,0);
           }
           else if (gg==69){
               EncodersControl(0.6,300,300,300,300,0,0,0,0);
               EncodersControl(0.6,450,-450,-450,450,0,0,0,0);
               EncodersControl(0.6,850,850,850,850,1,1,0,0);
               EncodersControl(0.3,900,900,900,900,1,1,0,0);
               EncodersControl(0.6,-950,-950,-950,-950,1,1,0,0);
               EncodersControl(1,915,-915,915,-915,1,1,0,0);
               gyroTurn(0.2,273);
               EncodersControl(0.6,-3750,-3750,-3750,-3750,0,0,0,0);
               EncodersControl(1,915,-915,915,-915,1,1,0,0);
               gyroTurn(0.2,182);
               EncodersControl(0.3,-500,-500,-500,-500,0,0,0,0);
               Servo1.setPosition(0.8);
               Servo2.setPosition(0.8);
               gyroHold(0.1,182,2);
               Servo1.setPosition(0.5);
               Servo2.setPosition(0.5);
               EncodersControl(0.8,850,850,850,850,0,0,0,0);
               Servo1.setPosition(0.6);
               Servo2.setPosition(0.6);
               EncodersControl(0.8,-1800,1800,-1800,1800,0,0,0,0);
               Servo1.setPosition(0.3);
               Servo2.setPosition(0.3);
               EncodersControl(1,-900,-900,-900,-900,0,0,0,0);
               EncodersControl(0.8,1600,1600,1600,1600,0,0,0,0);
               EncodersControl(0.6,-400,400,400,-400,0,0,0,0);
           }
           else if (gg==254){
               EncodersControl(0.6,300,300,300,300,0,0,0,0);
               EncodersControl(0.6,850,-850,-850,850,0,0,0,0);
               EncodersControl(0.6,850,850,850,850,1,1,0,0);
               EncodersControl(0.3,900,900,900,900,1,1,0,0);
               EncodersControl(0.6,-950,-950,-950,-950,1,1,0,0);
               EncodersControl(1,915,-915,915,-915,1,1,0,0);
               gyroTurn(0.2,272.5);
               EncodersControl(0.6,-4100,-4100,-4100,-4100,0,0,0,0);
               EncodersControl(1,915,-915,915,-915,0,0,0,0);
               gyroTurn(0.2,183);
               EncodersControl(0.3,-500,-500,-500,-500,0,0,0,0);
               Servo1.setPosition(0.8);
               Servo2.setPosition(0.8);
               gyroHold(0.1,182,2);
               Servo1.setPosition(0.5);
               Servo2.setPosition(0.5);
               EncodersControl(0.8,850,850,850,850,0,0,0,0);
               Servo1.setPosition(0.6);
               Servo2.setPosition(0.6);
               EncodersControl(0.8,-1800,1800,-1800,1800,0,0,0,0);
               Servo1.setPosition(0.3);
               Servo2.setPosition(0.3);
               EncodersControl(1,-1000,-1000,-1000,-1000,0,0,0,0);
               EncodersControl(0.8,1600,1600,1600,1600,0,0,0,0);
               EncodersControl(0.8,-400,400,400,-400,0,0,0,0);

           }

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

     public void EncodersControl(double speed, int leftForMM, int rightForMM, int leftBackMM, int rightBackMM, double rightMotorPower, double leftMotorPower,int RightEMM,int LeftEMM) {
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

            ForRight.setPower(Math.abs(speed));
            ForLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            RightE.setPower(Math.abs(speed));
            LeftE.setPower(Math.abs(speed));

            while (opModeIsActive() && (ForRight.isBusy() && ForLeft.isBusy() && BackRight.isBusy() && BackLeft.isBusy())) {

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

            RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
        public void gyroDrive ( double speed,double Espeed, int distance, double angle,int EMotors,double ShlomMotor){
            int newTargetForRight;
            int newTargetForLeft;
            int newTargetBackRight;
            int newTargetBackLeft;
            int moveCounts;
            int Right;
            int Left;
            double max;
            double error;
            double steer;
            double leftSpeed;
            double rightSpeed;

            if (opModeIsActive()) {
                moveCounts = distance;
                newTargetBackLeft=BackLeft.getCurrentPosition() +  moveCounts;
                newTargetBackRight=BackRight.getCurrentPosition() + moveCounts;
                newTargetForLeft=ForLeft.getCurrentPosition() + moveCounts;
                newTargetForRight=ForRight.getCurrentPosition() + moveCounts;
                Right=RightE.getTargetPosition()+EMotors;
                Left=LeftE.getTargetPosition()+EMotors;
                ForRight.setTargetPosition(newTargetForRight);
                ForLeft.setTargetPosition(newTargetForLeft);
                BackRight.setTargetPosition(newTargetBackRight);
                BackLeft.setTargetPosition(newTargetBackLeft);
                RightE.setTargetPosition(Right);
                LeftE.setTargetPosition(Left);
                RightE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ForRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ForLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                ForRight.setPower(speed);
                ForLeft.setPower(speed);
                BackRight.setPower(speed);
                BackLeft.setPower(speed);
                RightE.setPower(Espeed);
                LeftE.setPower(Espeed);
                RightMotor.setPower(ShlomMotor);
                LeftMotor.setPower(ShlomMotor);

                while (opModeIsActive() && (ForRight.isBusy() && ForLeft.isBusy() && BackRight.isBusy() && BackLeft.isBusy())) {

                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    ForRight.setPower(rightSpeed);
                    ForLeft.setPower(leftSpeed);
                    BackRight.setPower(rightSpeed);
                    BackLeft.setPower(leftSpeed);
                    RightMotor.setPower(0);
                    LeftMotor.setPower(0);
                    RightE.setPower(0);
                    LeftE.setPower(0);

                    telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newTargetForRight, newTargetForLeft, newTargetBackRight, newTargetBackLeft);
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            ForRight.getCurrentPosition(),
                            ForLeft.getCurrentPosition(),
                            BackRight.getCurrentPosition(),
                            BackLeft.getCurrentPosition());
                    telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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


        public void gyroTurn (  double speed, double angle) {
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
                telemetry.update();
            }
        }

        public void gyroHold( double speed, double angle, double holdTime) {
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {
                onHeading(speed, angle, P_TURN_COEFF);
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
        }

        boolean onHeading(double speed, double angle, double PCoeff) {
            double error;
            double steer;
            boolean onTarget = false;
            double leftSpeed;
            double rightSpeed;

            error = getError(angle);

            if (Math.abs(error) <= HEADING_THRESHOLD) {
                steer = 0.0;
                leftSpeed  = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            }else {
                steer = getSteer(error, PCoeff);
                rightSpeed  = speed * steer;
                leftSpeed   = -rightSpeed;
            }
            ForLeft.setPower(leftSpeed);
            ForRight.setPower(rightSpeed);
            BackLeft.setPower(leftSpeed);
            BackRight.setPower(rightSpeed);
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            return onTarget;
        }


        public double getError(double targetAngle) {

            double robotError;

            robotError = targetAngle - angles.firstAngle;
            while (robotError > 180)  robotError -= 360;
            while (robotError <= -180) robotError += 360;
            return robotError;
        }


        public double getSteer(double error, double PCoeff) {
            return Range.clip(error * PCoeff, -1, 1);
        }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(163, 108),
                    new Point(167, 132),
                    new Scalar(255,140,0), 2);

            Imgproc.rectangle(
                    input,
                    new Point(93, 108),
                    new Point(98, 132),
                    new Scalar(255,0,0), 2);

            double[] x = input.get(110, 165);
            double[] x2 = input.get(115,165);
            double[] x3 = input.get(120,165);
            double[] x4 = input.get(125,165);
            double[] x5 = input.get(130,165);

            double[] y = input.get(110,95);
            double[] y2 = input.get(115,95);
            double[] y3 = input.get(120,95);
            double[] y4 = input.get(125,95);
            double[] y5 = input.get(130,95);

            if ((y[0]+y[1]+y[2])<30||(y2[0]+y2[1]+y2[2])<30||(y3[0]+y3[1]+y3[2])<30||(y4[0]+y4[1]+y4[2])<30||(y5[0]+y5[1]+y5[2])<30){
                gg=69;
            }else if((x[0]+x[1]+x[2])<60||(x2[0]+x2[1]+x2[2])<60||(x3[0]+x3[1]+x3[2])<60||(x4[0]+x4[1]+x4[2])<60||(x5[0]+x5[1]+x5[2])<60){
                gg=420;
            }else {
                gg=254;
            }
            telemetry.addData("ClrY", y[0] + " " + y[1] + " " + y[2]);
            telemetry.addData("ClrX", x[0] + " " + x[1] + " " + x[2]);
            telemetry.addData("gg",gg);
            telemetry.update();
            return input;
        }
     }
}




