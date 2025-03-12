package drive.writtenCode.auto;


import static drive.writtenCode.auto.AutoBsk.STROBOT.SUB;
import static drive.writtenCode.auto.AutoBsk.STROBOT.SUB_INTER;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import drive.writtenCode.RobotMap;
import drive.writtenCode.controllers.ClawController;
import drive.writtenCode.controllers.ClawPositionController;
import drive.writtenCode.controllers.ClawRotateController;
import drive.writtenCode.controllers.CollectBrakeController;
import drive.writtenCode.controllers.FourbarController;
import drive.writtenCode.controllers.LinkageController;
import drive.writtenCode.controllers.LinkageSlidesController;
import drive.writtenCode.controllers.ScoreSystemController;
import drive.writtenCode.controllers.SlidesController;
import drive.writtenCode.pipelines.MihneaDetection;
import drive.writtenCode.pipelines.ShapeDetectionPipeline;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(group = "Auto")

public class InverseKinematics extends LinearOpMode {

    private Follower follower;
    final Pose startPose = new Pose(66,94,Math.toRadians(270));
    private OpenCvWebcam webcam;
    private MihneaDetection pipeline;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        FourbarController fourbar = new FourbarController(robot);
        LinkageController linkage = new LinkageController(robot);
        SlidesController slides = new SlidesController(robot);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        fourbar.currentStatus= FourbarController.FourbarStatus.SUB;


        slides.update(SlidesController.init_position,0);
        fourbar.update();
        follower.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new MihneaDetection(telemetry);
        webcam.setPipeline(pipeline);
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

        follower.setStartingPose(startPose);
        int target = 0;
        Point center = PersistentData.detected_center;
        double orientation = PersistentData.detected_angle;
        telemetry.addData("x", center.x);
        telemetry.addData("y", center.y);
        telemetry.addData("angle",orientation);
        telemetry.update();
        Pose aligned;
        if(center.x>320)
        {
            aligned = new Pose(startPose.getX() + (320-center.x)*0.03, startPose.getY(),startPose.getHeading());
        }
        else
        {
            aligned = new Pose(startPose.getX() + (320-center.x)*0.044, startPose.getY(),startPose.getHeading());
        }
        Path path;
        path= new Path(new BezierLine(new com.pedropathing.pathgen.Point(startPose),new com.pedropathing.pathgen.Point(aligned)));
        path.setConstantHeadingInterpolation(startPose.getHeading());
        path.setPathEndTranslationalConstraint(0.1);
        waitForStart();
        boolean ok=false;
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            if(isStopRequested()) return;
            // Obține coordonata și orientarea formei centermost din pipeline

            if (center != null) {
                telemetry.addData("Center Coordinate", String.format("(%.2f, %.2f)", center.x, center.y));
            } else {
                telemetry.addData("Center Coordinate", "Not detected");
            }

            if(ok==false && timer.seconds()>0.1) {
                follower.followPath(path,true);
                timer.reset();
                ok=true;
            }
//            if(timer.seconds()>0.2 && timer.seconds()<0.7)
//            {
//                robot.linkage.setPower(-1);
//            }
//            else
//            {
//                robot.linkage.setPower(0);
//            }
            if(timer.seconds()>1.5)
            {
                slides.currentStatus= SlidesController.SlidesStatus.AUTO_RUNTO;
                target=130*(int)center.y-18034;  //incremente mai mari-se duce mai putin
            }
            telemetry.addData("Orientation", String.format("%.2f", orientation));
            telemetry.addData("Detected X", aligned.getX());
            telemetry.addData("actual X", follower.getPose().getX());
            telemetry.addData("target", target);
            telemetry.addData("actual", robot.encoderSlides.getCurrentPosition());
            telemetry.update();
            follower.update();
            slides.update(robot.encoderSlides.getCurrentPosition(), target);
            fourbar.update();
        }
        webcam.stopStreaming();
    }

}