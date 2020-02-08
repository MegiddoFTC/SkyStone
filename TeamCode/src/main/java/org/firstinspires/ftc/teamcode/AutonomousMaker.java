package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;

@Disabled
@Autonomous(name = "AutonomousMaker")
public class AutonomousMaker extends LinearOpMode {
    private VuforiaSkyStone vuforiaSkyStone;
    ModernRoboticsI2cGyro gyro    = null;
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
    static final double WHEEL_DIAMETER_MM = 75;
    static final double DRIVE_GEAR_REDUCTION = 15;
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_MM * 3.1415);

        static final double     DRIVE_SPEED             = 0.7;
        static final double     TURN_SPEED              = 0.5;

        static final double     HEADING_THRESHOLD       = 1 ;
        static final double     P_TURN_COEFF            = 0.1;
        static final double     P_DRIVE_COEFF           = 0.15;


        @Override
        public void runOpMode() {
            CupStone = hardwareMap.get(Servo.class,"CupStone");
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
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
            gyro.calibrate();
            while (!isStopRequested() && gyro.isCalibrating())  {
                sleep(50);
                idle();
            }
            telemetry.addData(">", "Robot Ready.");
            telemetry.update();
            while (!isStarted()) {
                telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                telemetry.update();
            }
            gyro.resetZAxisIntegrator();

            waitForStart();
            while (opModeIsActive()) {
                if (isTargetVisible("Stone Target")) {
                    processTarget(vuforiaSkyStone.track("Stone Target"));
                } else {
                    telemetry.addData("No Targets Detected", "Targets are not visible.");
                }
            }
            vuforiaSkyStone.deactivate();
            vuforiaSkyStone.close();
            if(isTargetVisible("Stone Target")){}

            gyroDrive(DRIVE_SPEED, 48.0, 0.0);
            gyroTurn( TURN_SPEED, -45.0);
            gyroHold( TURN_SPEED, -45.0, 0.5);
            gyroDrive(DRIVE_SPEED, 12.0, -45.0);
            gyroTurn( TURN_SPEED,  45.0);
            gyroHold( TURN_SPEED,  45.0, 0.5);
            gyroTurn( TURN_SPEED,   0.0);
            gyroHold( TURN_SPEED,   0.0, 1.0);
            gyroDrive(DRIVE_SPEED,-48.0, 0.0);

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
            RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
        public void gyroDrive ( double speed, double distance, double angle) {
            int newTargetForRight;
            int newTargetForLeft;
            int newTargetBackRight;
            int newTargetBackLeft;
            int newTargetRightE;
            int newTargetLeftE;
            double newTargetRightMotor;
            double newTargetLeftMotor;
            int     moveCounts;
            double  max;
            double  error;
            double  steer;
            double  leftSpeed;
            double  rightSpeed;

            if (opModeIsActive()) {

                moveCounts = (int)(distance * COUNTS_PER_MM);
                newTargetBackLeft=BackLeft.getCurrentPosition() +  moveCounts;
                newTargetBackRight=BackRight.getCurrentPosition() + moveCounts;
                newTargetForLeft=ForLeft.getCurrentPosition() + moveCounts;
                newTargetForRight=ForRight.getCurrentPosition() + moveCounts;
                ForRight.setTargetPosition(newTargetForRight);
                ForLeft.setTargetPosition(newTargetForLeft);
                BackRight.setTargetPosition(newTargetBackRight);
                BackLeft.setTargetPosition(newTargetBackLeft);
                ForRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ForLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                ForRight.setPower(speed);
                ForLeft.setPower(speed);
                BackRight.setPower(speed);
                BackLeft.setPower(speed);

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
            double   error ;
            double   steer ;
            boolean  onTarget = false ;
            double leftSpeed;
            double rightSpeed;

            error = getError(angle);

            if (Math.abs(error) <= HEADING_THRESHOLD) {
                steer = 0.0;
                leftSpeed  = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            }
            else {
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

            robotError = targetAngle - gyro.getIntegratedZValue();
            while (robotError > 180)  robotError -= 360;
            while (robotError <= -180) robotError += 360;
            return robotError;
        }


        public double getSteer(double error, double PCoeff) {
            return Range.clip(error * PCoeff, -1, 1);
        }

    private void initVuforia() {
        vuforiaSkyStone.initialize(
                "",
                VuforiaLocalizer.CameraDirection.BACK,
                true,
                true,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                -90, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
    }


    private boolean isTargetVisible(String trackableName) {
        boolean isVisible;
        VuforiaBase.TrackingResults vuforiaResults = vuforiaSkyStone.track(trackableName);
        if (vuforiaResults.isVisible) {
            isVisible = true;
        } else {
            isVisible = false;
        }
        return isVisible;
    }


    private void processTarget(VuforiaBase.TrackingResults vuforiaResults) {
        telemetry.addData("Target Detected", vuforiaResults.name + " is visible.");
        telemetry.addData("X (mm)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.x, "MM"), 2)));
        telemetry.addData("Y (mm)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "MM"), 2)));
        telemetry.addData("Z (mm)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.z, "MM"), 2)));
        telemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2)));
    }

    private double displayValue(float originalValue, String units) {
        if (units.equals("CM")) return  originalValue / 10;
        else if (units.equals("M")) return originalValue / 1000;
        else if (units.equals("IN")) return originalValue / 25.4;
        else if (units.equals("FT")) return (originalValue / 25.4) / 12;
        else return originalValue;
    }
}




