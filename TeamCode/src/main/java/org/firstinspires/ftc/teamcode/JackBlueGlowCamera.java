package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
//@Disabled
@Autonomous(name = "JJ")
public class JackBlueGlowCamera extends LinearOpMode {
    private VuforiaSkyStone vuforiaSkyStone;
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
    @Override
    public void runOpMode() {
        vuforiaSkyStone = new VuforiaSkyStone();
        telemetry.addData("Status", "Initializing Vuforia. Please wait...");
        telemetry.update();
        initVuforia();
        vuforiaSkyStone.activate();
        telemetry.addData(">>", "Vuforia initialized, press start to continue...");
        telemetry.update();
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
        while (opModeIsActive()) {
            if (isTargetVisible("Stone Target")) {
                processTarget(vuforiaSkyStone.track("Stone Target"));
            } else {
                telemetry.addData("No Targets Detected", "Targets are not visible.");
            }
            vuforiaSkyStone.deactivate();
            vuforiaSkyStone.close();
            if(isTargetVisible("Stone Target")){}
                return;
        }

            EncodersControl(0.3,5000,5000,5000,5000,0,0);
            EncodersControl(0.3,-5000,5000,-5000,5000,0,0);

    }
    public void EncodersControl(double speed, int leftForMM, int rightForMM, int leftBackMM, int rightBackMM, double rightMotorPower, double leftMotorPower) {
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

            ForRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ForLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            ForRight.setPower(Math.abs(speed));
            ForLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));

            while ((opModeIsActive() && (ForRight.isBusy() && ForLeft.isBusy() && BackRight.isBusy() && BackLeft.isBusy() && RightMotor.isBusy() && LeftMotor.isBusy() && RightE.isBusy() && LeftE.isBusy())) ||(opModeIsActive()&&isTargetVisible("Stone Target")) ) {

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
    private void initVuforia() {
        // Rotate phone -90 so back camera faces "forward" direction on robot.
        // Assumes landscape orientation
        vuforiaSkyStone.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                true, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                -90, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
    }

    /**
     * Check to see if the target is visible.
     */
    private boolean isTargetVisible(String trackableName) {
        //VuforiaSkyStone vuforiaResult;
        boolean isVisible;
        //vuforiaResult;
        // Get vuforia results for target.
        VuforiaBase.TrackingResults vuforiaResults = vuforiaSkyStone.track(trackableName);
        // Is this target visible?
        if (vuforiaResults.isVisible) {
            isVisible = true;
        } else {
            isVisible = false;
        }
        return isVisible;
    }

    /**
     * This function displays location on the field and rotation about the Z
     * axis on the field. It uses results from the isTargetVisible function.
     */
    private void processTarget(VuforiaBase.TrackingResults vuforiaResults) {
        // Display the target name.
        telemetry.addData("Target Detected", vuforiaResults.name + " is visible.");
        telemetry.addData("X (mm)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.x, "MM"), 2)));
        telemetry.addData("Y (mm)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "MM"), 2)));
        telemetry.addData("Z (mm)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.z, "MM"), 2)));
        telemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2)));
    }

    /**
     * By default, distances are returned in millimeters by Vuforia.
     * Convert to other distance units (CM, M, IN, and FT).
     */
    private double displayValue(float originalValue, String units) {
        if (units.equals("CM")) return  originalValue / 10;
        else if (units.equals("M")) return originalValue / 1000;
        else if (units.equals("IN")) return originalValue / 25.4;
        else if (units.equals("FT")) return (originalValue / 25.4) / 12;
        else return originalValue;
    }
}

