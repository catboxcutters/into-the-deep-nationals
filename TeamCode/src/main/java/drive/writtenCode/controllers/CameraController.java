package drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import drive.writtenCode.pipelines.edge_pipeline;

public class CameraController {

    private OpenCvWebcam webcam;
    private edge_pipeline pipeline;

    // Constructor to initialize the camera
    public CameraController(HardwareMap hardwareMap, String webcamName) {
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // Initialize the edge detection pipeline
        pipeline = new edge_pipeline();
        webcam.setPipeline(pipeline);
    }

    // Start the camera with asynchronous initialization
    public void startCamera() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.err.println("Error Opening Camera: Error code " + errorCode);
            }
        });
    }

    // Stop the camera
    public void stopCamera() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.stopRecordingPipeline();
        }
    }

    // Get the orientation angle from the pipeline
    public double getOrientationAngle() {
        return pipeline.getOrientationAngle();
    }

    // Enable or disable the masked (edge-detected) image in the pipeline
    public void setShowMaskedImage(boolean showMaskedImage) {
        pipeline.setShowMaskedImage(showMaskedImage);
    }

    // Toggle between detecting blue or yellow lines
    public void toggleDetectionColor() {
        boolean isDetectingBlue = pipeline.isDetectingBlue();
        pipeline.setDetectBlue(!isDetectingBlue);  // Toggle the current detection color
        System.out.println("Detection color toggled to: " + (isDetectingBlue ? "Yellow" : "Blue"));
    }
    public String getCurrentDetectionColor() {
        return pipeline.isDetectingBlue() ? "Blue" : "Yellow";
    }
}
