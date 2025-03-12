package drive.writtenCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import drive.writtenCode.auto.PersistentData;
import drive.writtenCode.controllers.ClawRotateController;
import drive.writtenCode.controllers.LinkageController;

import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import drive.writtenCode.controllers.FourbarController;
import drive.writtenCode.controllers.LinkageController;
import drive.writtenCode.controllers.SlidesController;
import drive.writtenCode.pipelines.MihneaDetection;
import drive.writtenCode.pipelines.ShapeDetectionPipeline;
import org.opencv.core.Point;

@TeleOp(name = "teleoptest2", group = "Test")
public class teleoptest2 extends LinearOpMode {
    private VisionPortal visionPortal;
    private OpenCvWebcam webcam;
    private MihneaDetection pipeline;


    @Override
    public void runOpMode() throws InterruptedException {
//        visionPortal= VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboard.startCameraStream(visionPortal,5);
        RobotMap robot= new RobotMap(hardwareMap);
//        robot.encoderLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        robot.encoderSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinkageController linkageController = new LinkageController(robot);
        FourbarController fourbarController = new FourbarController(robot);
        ClawRotateController clawRotateController = new ClawRotateController(robot);
//        SlidesController slidesController = new SlidesController(robot);

        clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.RUNTO;
        fourbarController.currentStatus= FourbarController.FourbarStatus.DETECTION;

//        linkageController.update(LinkageController.init_position,0);
//        slidesController.update(SlidesController.init_position,0);
        fourbarController.update();
        // Inițializarea camerei
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Setarea pipeline-ului nostru de detecție a formelor
        pipeline = new MihneaDetection(telemetry);
        webcam.setPipeline(pipeline);

        // Deschide camera asincron și pornește streaming-ul
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Utilizăm dimensiunea definită în pipeline (1280x800)
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error Opening Camera", "Error code: " + errorCode);
                telemetry.update();
            }
        });
        waitForStart();
        while (opModeIsActive()) {
            if(isStopRequested()) return;
            // Obține coordonata și orientarea formei centermost din pipeline
            linkageController.currentStatus= LinkageController.LinkageStatus.DETECTION;
//            slidesController.currentStatus= SlidesController.SlidesStatus.INIT;

            if(isStopRequested()) return;
            // Obține coordonata și orientarea formei centermost din pipeline
            Point center = pipeline.getClosestPoint();
            double orientation = pipeline.getClosestAngle();
            double claw_rotate_target = 0.52;
            if(orientation>0 && orientation<=90)
            {
                clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.RUNTO;
                claw_rotate_target = 0.003*orientation + 0.52;
            }
            else if(orientation>90 && orientation<=180)
            {
                clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.RUNTO;
                claw_rotate_target = 0.00322*orientation - 0.0596;
            }
            if (center != null) {
                telemetry.addData("Center Coordinate", String.format("(%.2f, %.2f)", center.x, center.y));
            } else {
                telemetry.addData("Center Coordinate", "Not detected");
            }
            telemetry.addData("Orientation", String.format("%.2f", orientation));
            telemetry.addData("EncoderSlides", robot.encoderSlides.getCurrentPosition());
            telemetry.addData("linkageencoder", robot.encoderLinkage.getCurrentPosition());
//            telemetry.addData("Slides_target", slidesController);
//            telemetry.addData("Shape Info", pipeline.getTelemetryData());
            telemetry.update();

            PersistentData.detected_center=center;
            PersistentData.detected_angle=orientation;
            linkageController.update(robot.encoderLinkage.getCurrentPosition(),0);
            clawRotateController.update(claw_rotate_target,0);
//            slidesController.update(robot.encoderSlides.getCurrentPosition(),0);
            fourbarController.update();
        }

        // Oprește streaming-ul camerei când opmode-ul se încheie
        webcam.stopStreaming();
    }
}
