package drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import drive.writtenCode.pipelines.edge_pipeline;

@TeleOp(name = "Edge Pipeline Opmode", group = "Test")
public class EdgePipelineOpmode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private edge_pipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline
        pipeline = new edge_pipeline();
        webcam.setPipeline(pipeline);

        // Open the camera device asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error Opening Camera", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Get the angle from the pipeline
            double angle = pipeline.getOrientationAngle();

            // Print the angle to telemetry
            telemetry.addData("Detected Angle", String.format("%.2f degrees", angle));
            telemetry.update();

            sleep(50); // Small delay to prevent overloading telemetry
        }

        // Stop the webcam streaming when opMode ends
        webcam.stopStreaming();
    }
}
