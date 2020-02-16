package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Disabled
@TeleOp(name = "vuforia example",group = "Iterative Opmode")
public class VurfiaExample extends LinearOpMode {
    private VuforiaSkyStone vuforiaSkyStone;

    @Override
    public void runOpMode() {
        vuforiaSkyStone = new VuforiaSkyStone();

        // Initialize Vuforia
        telemetry.addData("Status", "Initializing Vuforia. Please wait...");
        telemetry.update();
        initVuforia();
        // Activate here for camera preview.
        vuforiaSkyStone.activate();
        telemetry.addData(">>", "Vuforia initialized, press start to continue...");
        telemetry.update();

        waitForStart();
        /*
        while (opModeIsActive()) {
            // check if there is sky stone
            if (isTargetVisible("SkyStone3")) {
                // show everything about target
                processTarget(vuforiaSkyStone.track("SkyStone3"));
//                telemetry.addData("bla", "bla bla");
            } else {
                telemetry.addData("No Targets Detected", "Targets are not visible.");
            }
            telemetry.update();
        }*/

        while (opModeIsActive()) {
            if (isTargetVisible("SyStone2")){
                telemetry.addData("nla", "bla bla\n");
            }
        }

        vuforiaSkyStone.deactivate();
        vuforiaSkyStone.close();
//        if(isTargetVisible("SkyStone")){
//
//        }
    }

    /**
     * start vuforia
     */
    private void initVuforia() {
        // Rotate phone -90 so back camera faces "forward" direction on robot.
        // Assumes landscape orientation
        vuforiaSkyStone.initialize(
                "AX1u28H/////AAABmVnaN1DibUP2qqhbRQ9upMwecZe4HqbIyDF6mRqT02lkLq4BQ03rzHTBlDcKGxQZz87xMS2I/w30Frof05wrrN6mihivWgIschkjAyZvJOyy+3R3cQVkBZ0Jz4Qc7JjvknA07b5iJVp3AOQWYPqunskYCEQ+OSHZsuiu8Nc0O1pOc1zXeIxaxPiy/jhpX3zPg3SvC8G0fpvZWY9FgOggWa2BpE5rMjFF5orX5OZUHa6pcECIyIh1oBJ9EvntYt4ogBhCiTrSe0oF8ecrT80rRs1ACHz7cRIPz25f5xwYEAxYBznbHwwM17PgfchDVjrJ/vVyiKh6kTJZawZp5Nz2rMXIP1Hznc6SXgrfeQvebOCe", // vuforiaLicenseKey
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
