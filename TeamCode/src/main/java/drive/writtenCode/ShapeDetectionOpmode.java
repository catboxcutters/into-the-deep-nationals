package drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import drive.writtenCode.pipelines.ShapeDetectionPipeline;
import org.opencv.core.Point;

@TeleOp(name = "Shape Detection Opmode", group = "Test")
public class ShapeDetectionOpmode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private ShapeDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        // Inițializarea camerei
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Setarea pipeline-ului nostru de detecție a formelor
        pipeline = new ShapeDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        // Deschide camera asincron și pornește streaming-ul
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Utilizăm dimensiunea definită în pipeline (1280x800)
                webcam.startStreaming(1920, 1200, OpenCvCameraRotation.UPRIGHT);
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
            if(isStopRequested()) return;
            // Obține coordonata și orientarea formei centermost din pipeline
            Point center = pipeline.getCentermostCoordinate();
            double orientation = pipeline.getCentermostOrientation();

            if (center != null) {
                telemetry.addData("Center Coordinate", String.format("(%.2f, %.2f)", center.x, center.y));
            } else {
                telemetry.addData("Center Coordinate", "Not detected");
            }
            telemetry.addData("Orientation", String.format("%.2f", orientation));
            telemetry.addData("Shape Info", pipeline.getTelemetryData());
            telemetry.update();

            sleep(1000); // Pauză scurtă pentru a evita suprasolicitarea telemetriei
        }

        // Oprește streaming-ul camerei când opmode-ul se încheie
        webcam.stopStreaming();
    }
}
