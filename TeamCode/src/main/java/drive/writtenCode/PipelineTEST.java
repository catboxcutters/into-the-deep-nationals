package drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import drive.writtenCode.controllers.CameraController;

@TeleOp(name = "Camera Controller Test", group = "Test")
public class PipelineTEST extends LinearOpMode {

    private CameraController cameraController;

    @Override
    public void runOpMode() {
        // Initialize the CameraController
        cameraController = new CameraController(hardwareMap, "Webcam 1");

        // Start the camera
        cameraController.startCamera();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Get the detected angle from the pipeline
            double angle = cameraController.getOrientationAngle();

            // Display the angle on telemetry
            telemetry.addData("Detected Orientation Angle", "%.2f degrees", angle);
            telemetry.update();

            sleep(50); // Prevent spamming telemetry updates
        }

        // Stop the camera when OpMode ends
        cameraController.stopCamera();
    }
}
