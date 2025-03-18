package drive.writtenCode.auto;


import static drive.writtenCode.auto.AutoBsk.STROBOT.DETECT;
import static drive.writtenCode.auto.AutoBsk.STROBOT.END_AUTO;
import static drive.writtenCode.auto.AutoBsk.STROBOT.SCORE_SUB;
import static drive.writtenCode.auto.AutoBsk.STROBOT.SLIDES_SUB;
import static drive.writtenCode.auto.AutoBsk.STROBOT.SUB;
import static drive.writtenCode.auto.AutoBsk.STROBOT.SUB_COLLECT;
import static drive.writtenCode.auto.AutoBsk.STROBOT.SUB_INTER;

import android.bluetooth.le.ScanSettings;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import drive.writtenCode.RobotMap;
import drive.writtenCode.controllers.ClawController;
import drive.writtenCode.controllers.ClawPositionController;
import drive.writtenCode.controllers.ClawRotateController;
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

public class AutoBsk extends LinearOpMode {


    public static boolean isBetween(int value, int low, int high) {
        return value >= low && value <= high; // Change operators if exclusive
    }


    enum STROBOT
    {
        START,
        PLACE_PRELOAD, SUB,
        PICKUP1, GRAB_PICKUP1, PICKUP2, GRAB_PICKUP2, PICKUP3, GRAB_PICKUP3, END_AUTO,
        SCORE_PICKUP1, SCORE_PICKUP2, SCORE_PICKUP3, SCORE_INTER1, SCORE_INTER2, SCORE_INTER3, INTER_PRELOAD,PARK, GRAB_PICKUP1_INTER,
        SUB_INTER, SUB_COLLECT, SCORE_INTER_SUB, SCORE_SUB, DETECT, SLIDES_SUB;
    }
    List<DetectedPoint> detectedPoints = new ArrayList<>();
    double angle = 90;

    class DetectedPoint {
        org.opencv.core.Point point;
        double angle;
        double minDist;

        public DetectedPoint(org.opencv.core.Point point, double angle, double minDist) {
            this.point = point;
            this.angle = angle;
            this.minDist = minDist;
        }
    }
    org.opencv.core.Point lastDetectedPoint = new org.opencv.core.Point(-1, -1);
    double lastDetectedAngle = 0;
    double lastDetectedMinDist = Double.MAX_VALUE;
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerLeave = new ElapsedTime();
    ElapsedTime TimerPlacePreload = new ElapsedTime();
    ElapsedTime TimerPickup = new ElapsedTime();
    ElapsedTime TimerScore = new ElapsedTime();
    ElapsedTime TimerCollect = new ElapsedTime();
    ElapsedTime TimerDetection = new ElapsedTime();
    ElapsedTime TimerSub = new ElapsedTime();
    private double time_place_preload = 1.3;
    private double time_leave_preload = 0.5;
    private Follower follower;

    int linkage_target_position = 0;
    double claw_rotate_target = 0;
    int slides_target_position = 0;

    private Path sub,sub2,sub3,sub4,score1,score2,score3,score4,scorePreload,scorePreloadInter ,pickup1,scorePickup1Inter,scorePickup1,pickup2,scorePickup2Inter,scorePickup2,pickup3,scorePickup3Inter,scorePickup3,park;
    private PathChain scorePickup1Chain;
    public static double tunex=0;
    public static double tuney=0;
    final Pose startPose = new Pose(10, 111, Math.toRadians(270));
    final Pose preloadPose = new Pose(20, 132, Math.toRadians(327));
    final Pose interPreload = new Pose(20, 130, Math.toRadians(325));
    final Pose interPose = new Pose(22,130, Math.toRadians(327));
    final Pose scoreSubPose = new Pose(22,133.5,Math.toRadians(327));
    final Pose scorePose = new Pose(19, 132, Math.toRadians(327));
    final Pose scorePose2 = new Pose(19,132, Math.toRadians(327)); //4 129.5
    final Pose scorePose3 = new Pose(20,132, Math.toRadians(327));
//    final Pose pickup1InterPose = new Pose(29,118,Math.toRadians(0));
    final Pose pickup1Pose = new Pose(37.2, 123.3, Math.toRadians(347));
    final Pose pickup2Pose = new Pose(37,132.3,Math.toRadians(0)); //41 19
    final Pose pickup3PoseInter = new Pose(30,130,Math.toRadians(90));
    final Pose pickup3Pose = new Pose(39,133,Math.toRadians(22));
    final Pose subPoseInter = new Pose(60,110,Math.toRadians(270));

    double[]Sx =PersistentData.subX;
    int[]Ss = PersistentData.slidesPosition; //not used
    final Pose subPose = new Pose(Sx[1],92,Math.toRadians(270)), subPose2 = new Pose(Sx[2],92,Math.toRadians(270))
            , subPose3 = new Pose(Sx[3],92,Math.toRadians(270)), subPose4 = new Pose(Sx[4],92,Math.toRadians(270));

    final Point subPoint = new Point(Sx[1],113), subPoint2 = new Point(Sx[2],113)
            , subPoint3 = new Point(Sx[3],113), subPoint4 = new Point(Sx[4],113);




    final Pose parkPose =new Pose(35,120, Math.toRadians(270));
    private OpenCvWebcam webcam;
    private MihneaDetection pipeline;
    int cycle = 1;
    int i=1;
    int nr_frames=1;
    double nr_slides=1;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        DigitalChannel beam = robot.beam;
        ClawController claw = new ClawController(robot);
        ClawPositionController clawPosition = new ClawPositionController(robot);
        ClawRotateController clawRotate = new ClawRotateController(robot);
        FourbarController fourbar = new FourbarController(robot);
        LinkageController linkage = new LinkageController(robot);
        SlidesController slides = new SlidesController(robot);
        ScoreSystemController scoreSystem = new ScoreSystemController(claw,clawRotate,fourbar,clawPosition,linkage);
        LinkageSlidesController linkageSlides = new LinkageSlidesController(linkage,slides);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        sub = new Path(new BezierCurve(new Point(scorePose3), subPoint,new Point(subPose)));
        sub.setTangentHeadingInterpolation();
        sub.setPathEndTimeoutConstraint(0.7);
        sub2 = new Path(new BezierCurve(new Point(scorePose3), subPoint2,new Point(subPose2)));
        sub2.setTangentHeadingInterpolation();
        sub2.setPathEndTimeoutConstraint(0.7);
        sub3 = new Path(new BezierCurve(new Point(scorePose3), subPoint3,new Point(subPose3)));
        sub3.setTangentHeadingInterpolation();
        sub3.setPathEndTimeoutConstraint(0.7);
        sub4 = new Path(new BezierCurve(new Point(scorePose3), subPoint4,new Point(subPose4)));
        sub4.setTangentHeadingInterpolation();
        sub4.setPathEndTimeoutConstraint(0.7);


        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.INIT_AUTO;

        scoreSystem.update(robot.encoderClaw.getVoltage());
        linkageSlides.update();
        claw.update();
        clawPosition.update();
        clawRotate.update(ClawRotateController.init_position,0);
        fourbar.update();
        follower.update();
        linkage.update(LinkageController.init_position,linkage_target_position);
        slides.update(SlidesController.init_position,slides_target_position);

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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error Opening Camera", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        boolean ok=false;
        follower.setStartingPose(startPose);
        STROBOT status = STROBOT.START;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int slides_current_position = robot.encoderSlides.getCurrentPosition();
            int linkage_current_position = linkage.encoderLinkage.getCurrentPosition();
            switch (status) {
                case START: {
                    AutoTimer.reset();
                    TimerPlacePreload.reset();
                    follower.holdPoint(preloadPose);
                    status = STROBOT.INTER_PRELOAD;
                    break;
                }
                case INTER_PRELOAD:
                {
                    if(TimerPlacePreload.seconds()>0.4)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH_AUTO;
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.MINUS2;
                        fourbar.currentStatus= FourbarController.FourbarStatus.SCORE_AUTO;
                        status=STROBOT.PLACE_PRELOAD;
                    }
                    break;
                }
                case PLACE_PRELOAD: {
                    if(TimerPlacePreload.seconds() >1)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE_AUTO;
//                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                    }
                    if(TimerPlacePreload.seconds() > 1.1)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerPlacePreload.seconds() > 1.2)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                    }
                    if(TimerPlacePreload.seconds()>1.3)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    }
                    if(TimerPlacePreload.seconds()>1.3)
                    {
                        TimerPickup.reset();
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        follower.holdPoint(pickup1Pose);
                        status=STROBOT.GRAB_PICKUP1;
                    }
                    break;
                }
                case GRAB_PICKUP1:
                {
                    if(TimerPickup.seconds()>0.2 && TimerPickup.seconds()<0.5)
                    {
                        linkage.currentStatus= LinkageController.LinkageStatus.COLLECT;
                        slides.currentStatus= SlidesController.SlidesStatus.AUTO;
                    }
                    if(TimerPickup.seconds()>0.65)
                    {
                        slides.currentStatus = SlidesController.SlidesStatus.AUTO_RUNTO;
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.PICKUP1;
                        slides_target_position = -16800;
                    }
                    if(TimerPickup.seconds()>1.3 && claw.currentStatus!= ClawController.ClawStatus.CLOSE)
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 1.5) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                    }
                    if(TimerPickup.seconds()>1.7)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    }
                    if(TimerPickup.seconds()>1.7)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                    }
                    if(TimerPickup.seconds()>1.7)
                    {
                        follower.holdPoint(scorePose);
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        TimerScore.reset();
                        status=STROBOT.SCORE_INTER1;
                    }
                    break;
                }
                case SCORE_INTER1:
                {
                    if(TimerScore.seconds()>0.2)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.MINUS2;
                    }
                    if(TimerScore.seconds()>0.4)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH_AUTO;
                        status=STROBOT.SCORE_PICKUP1;
                    }
                    break;
                }
                case SCORE_PICKUP1:
                {
                    if(TimerScore.seconds() >0.9)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE_AUTO;
//                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                    }
                    if(TimerScore.seconds() > 1.15)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerScore.seconds() > 1.2)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                    }
                    if(TimerScore.seconds()>1.3)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    }

                    if(TimerScore.seconds()>1.3)
                    {
                        TimerPickup.reset();
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        follower.holdPoint(pickup2Pose);
                        status=STROBOT.GRAB_PICKUP2;
//                        status=SUB_INTER;
                    }

                    break;
                }
                case GRAB_PICKUP2:
                {
                    if(TimerPickup.seconds()>0.2)
                    {
                        linkage.currentStatus= LinkageController.LinkageStatus.COLLECT;
                        slides.currentStatus= SlidesController.SlidesStatus.AUTO;
                    }
                    if(TimerPickup.seconds()>0.8)
                    {
                        slides.currentStatus = SlidesController.SlidesStatus.AUTO_RUNTO;
                        slides_target_position = -18500;
                    }
                    if(TimerPickup.seconds()>1.2 && claw.currentStatus!= ClawController.ClawStatus.CLOSE)
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 1.3) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                    }
                    if(TimerPickup.seconds()>1.5)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    }
                    if(TimerPickup.seconds()>1.5)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                    }
                    if(TimerPickup.seconds()>1.5)
                    {
                        follower.holdPoint(scorePose2);
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        TimerScore.reset();
                        status=STROBOT.SCORE_INTER2;
                    }
                    break;
                }
                case SCORE_INTER2:
                {
                    if(TimerScore.seconds()>0.2)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.MINUS2;
                    }
                    if(TimerScore.seconds()>0.4)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH_AUTO;
                        status=STROBOT.SCORE_PICKUP2;
                    }
                    break;
                }
                case SCORE_PICKUP2:
                {
                    if(TimerScore.seconds() >0.9)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE_AUTO;
//                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                    }
                    if(TimerScore.seconds() > 1.1)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerScore.seconds() > 1.15)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                    }
                    if(TimerScore.seconds()>1.25)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    }
                    if(TimerScore.seconds()>1.4)
                    {
                        TimerPickup.reset();
                        linkage.currentStatus= LinkageController.LinkageStatus.COLLECT;
                        slides.currentStatus= SlidesController.SlidesStatus.AUTO;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.PICKUP3;
                        follower.holdPoint(pickup3Pose);
                        status=STROBOT.GRAB_PICKUP3;
                    }
                    break;
                }
                case GRAB_PICKUP3:
                {
                    if(TimerPickup.seconds()>0.8)
                    {
                        slides.currentStatus = SlidesController.SlidesStatus.AUTO_RUNTO;
                        slides_target_position = -20400;
                    }
                    if(TimerPickup.seconds()>1.4 && claw.currentStatus!= ClawController.ClawStatus.CLOSE)
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 1.5) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                    }
                    if(TimerPickup.seconds()>1.7)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    }
                    if(TimerPickup.seconds()>1.7)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                    }
                    if(TimerPickup.seconds()>1.7)
                    {
                        follower.holdPoint(scorePose3);
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        TimerScore.reset();
                        status=STROBOT.SCORE_INTER3;
                    }
                    break;
                }
                case SCORE_INTER3:
                {
                    if(TimerScore.seconds()>0.2)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.MINUS2;
                    }
                    if(TimerScore.seconds()>0.4)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH_AUTO;
                        status=STROBOT.SCORE_PICKUP3;
                    }
                    break;
                }
                case SCORE_PICKUP3:
                {
                    if(TimerScore.seconds() >0.9)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE_AUTO;
//                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                    }
                    if(TimerScore.seconds() > 1.1)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerScore.seconds() > 1.25)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                    }
                    if(TimerScore.seconds()>1.35)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    }
                    if(TimerScore.seconds()>1.35)
                    {
                        status=SUB_INTER;
                    }
                    break;
                }
                case SUB_INTER:
                {
                    switch(cycle)
                    {
                        case 1:
                        {
                            follower.followPath(sub,true);
                            status=SUB;
                            break;
                        }
                        case 2:
                        {
                            follower.followPath(sub2,true);
                            status=SUB;
                            break;
                        }
                        case 3:
                        {
                            follower.followPath(sub3,true);
                            status=SUB;
                            break;
                        }
                        case 4:
                        {
//                            follower.followPath(sub4,true);
                            status=SUB;
                            break;
                        }
                    }
                    ok=false;
                    TimerCollect.reset();
                    break;
                }
                case SUB:
                {
                    if(TimerScore.seconds()>0.6)
                    {
                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerCollect.seconds()>1.3 && ok==false)
                    {
                        ok=true;
                        switch(cycle) {
                            case 1:
                                follower.holdPoint(subPose);
                                break;
                            case 2:
                                follower.holdPoint(subPose2);
                                break;
                            case 3:
                                follower.holdPoint(subPose3);
                                break;
                            case 4:
//                                follower.holdPoint(subPose4);
                                break;
                        }
                    }
                    if(cycle!=4) {
                        if (TimerCollect.seconds() > 1.35) {
                            linkage.currentStatus = LinkageController.LinkageStatus.DETECTION;
                            fourbar.currentStatus = FourbarController.FourbarStatus.DETECTION;
                        }
                        if (TimerCollect.seconds() > 1.7) {
                            slides.currentStatus = SlidesController.SlidesStatus.DETECTION;
                        }
                        if (TimerCollect.seconds() > 2.6) {
                            TimerDetection.reset();
                            i = 1;
                            status = DETECT;
                        }
                    }
                    break;
                }
                case DETECT:
                {
                    if(i<=nr_frames && !follower.isBusy())
                    {
                        org.opencv.core.Point point = pipeline.getClosestPoint();
                        double angle = pipeline.getClosestAngle();
                        double min_dist = pipeline.getMinimumDistance();
                        if (point != null && point.x!=-1 && point.y>32 ) {
//                            boolean isNewDetection = (point.x != lastDetectedPoint.x || point.y != lastDetectedPoint.y) || (angle!=lastDetectedAngle) ||  (lastDetectedMinDist!=min_dist);
//                            if(isNewDetection) {
                                detectedPoints.add(new DetectedPoint(point, angle, min_dist));
                                // Sort list by minimum distance
                                detectedPoints.sort(Comparator.comparingDouble(dp -> dp.minDist));
                                telemetry.addData("found new", i);
                                lastDetectedPoint=point;
                                lastDetectedAngle=angle;
                                lastDetectedMinDist=min_dist;
                                i++;
//                            }
                        }
                    }
                    else
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus= LinkageController.LinkageStatus.COLLECT;
                        status=SUB_COLLECT;
                    }
                    if(TimerDetection.seconds()>1.2)
                    {
                        Pose current_pose=follower.getPose();
                        Pose failsafe = new Pose(current_pose.getX() - 5, current_pose.getY(),current_pose.getHeading());
                        Path failsafe_path;
                        failsafe_path= new Path(new BezierLine(new Point(current_pose),new Point(failsafe)));
                        failsafe_path.setConstantHeadingInterpolation(current_pose.getHeading());
                        failsafe_path.setPathEndTranslationalConstraint(0.1);
                        follower.followPath(failsafe_path);
                        TimerDetection.reset();
                    }
                    if(AutoTimer.seconds()>29)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                        status=END_AUTO;
                    }
                    break;
                }
                case SUB_COLLECT:
                {
                        org.opencv.core.Point center = detectedPoints.get(0).point;
                        double centerY = center.y;
                        if (centerY >= 281 && centerY < 316) {
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.BACKWARDS;
                        } else if (centerY >= 247 && centerY < 281) {
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.BACKWARDS;
                        } else if (centerY >= 215 && centerY < 247) {
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.BACKWARDS;
                        } else if (centerY >= 183 && centerY < 215) {
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.BACKWARDS;
                        }


                        angle = detectedPoints.get(0).angle;
                        Pose current_pose = follower.getPose();
                        Pose aligned;
                        if (center.x > 320) {
                            aligned = new Pose(current_pose.getX() + (320 - center.x) * 0.035, current_pose.getY(), current_pose.getHeading());
                        } else {
                            aligned = new Pose(current_pose.getX() + (320 - center.x) * 0.035, current_pose.getY(), current_pose.getHeading());
                        }
                        Path path;
                        path = new Path(new BezierLine(new Point(current_pose), new Point(aligned)));
                        path.setConstantHeadingInterpolation(current_pose.getHeading());
                        path.setPathEndTranslationalConstraint(0.1);
//                    if(center.y<200) {
//                        slides_target_position = 75 * (int) center.y - 20034;
//                    }
//                    if(center.y>200)
//                    {
//                        slides_target_position = 83 * (int) center.y - 20034;
//                    }
                        // Extract the y value
//                    if (centerY >= 281 && centerY < 316) {
//                        slides_target_position = 0;
//                        nr_slides = 1;
//                    } else if (centerY >= 247 && centerY < 281) {
//                        slides_target_position = -3400; //0
//                        nr_slides = 2;
//                    } else if (centerY >= 215 && centerY < 247) {
//                        slides_target_position = -7100; //-3400
//                        nr_slides = 3; //2700
//                    } else if (centerY >= 183 && centerY < 215) {
//                        slides_target_position = -8700; //7100
//                        nr_slides = 4;
//                    } else if (centerY >= 156 && centerY < 183) {
//                        slides_target_position = -11000; //8700
//                        nr_slides = 5;
//                    } else if (centerY >= 127 && centerY < 156) {
//                        slides_target_position = -14000; //11000
//                        nr_slides = 6;
//                    } else if (centerY >= 100 && centerY < 127) {
//                        slides_target_position = -17500; //14000
//                        nr_slides = 7;
//                    } else if (centerY >= 73 && centerY < 100) {
//                        slides_target_position = -19150; //17500
//                        nr_slides = 8;
//                    } else if (centerY >= 62 && centerY < 73) {
//                        slides_target_position = -20800; //19150
//                        nr_slides = 8.5;
//                    } else if (centerY >= 50 && centerY < 62) {
//                        slides_target_position = -21750; //20800
//                        nr_slides = 9;
//                    } else if (centerY >= 38 && centerY < 50) {
//                        slides_target_position = -22700; //21750
//                        nr_slides = 9.5;
//                    } else if (centerY >= 27 && centerY < 38) {
//                        slides_target_position = -24700; //22700
//                        nr_slides = 10;
//                    } else if (centerY >= 0 && centerY < 27) {
//                        slides_target_position = -24700;
//                        nr_slides = 11;
//                    }
                    if (centerY >= 260) {
                        slides_target_position = 0;
                        nr_slides = 1;
                    } else if (centerY >= 227 && centerY < 260) {
                        slides_target_position = -3700; //0
                        nr_slides = 2;
                    } else if (centerY >= 195 && centerY < 227) {
                        slides_target_position = -7400; //-3400
                        nr_slides = 3; //2700
                    } else if (centerY >= 163 && centerY < 195) {
                        slides_target_position = -9000; //7100
                        nr_slides = 4;
                    } else if (centerY >= 133 && centerY < 163) {
                        slides_target_position = -11300; //8700
                        nr_slides = 5;
                    } else if (centerY >= 103 && centerY < 133) {
                        slides_target_position = -14300; //11000
                        nr_slides = 6;
                    } else if (centerY >= 73 && centerY < 103) {
                        slides_target_position = -17800; //14000
                        nr_slides = 7;
                    } else if (centerY >= 43 && centerY < 73) {
                        slides_target_position = -19450; //17500
                        nr_slides = 8;
                    } else if (centerY >= 28 && centerY < 43) {
                        slides_target_position = -21100; //19150
                        nr_slides = 8.5;
                    } else if (centerY >= 13 && centerY < 28) {
                        slides_target_position = -22050; //20800
                        nr_slides = 9;
                    }
//                    else if (centerY >= 38 && centerY < 50) {
//                        slides_target_position = -22700; //21750
//                        nr_slides = 9.5;
//                    } else if (centerY >= 27 && centerY < 38) {
//                        slides_target_position = -24700; //22700
//                        nr_slides = 10;
//                    } else if (centerY >= 0 && centerY < 27) {
//                        slides_target_position = -24700;
//                        nr_slides = 11;
//                    }

                        follower.followPath(path);
                        TimerSub.reset();
                        status = SLIDES_SUB;

                    break;
                }
                case SLIDES_SUB:
                {
                    if(TimerSub.seconds()>0.4 && claw.currentStatus== ClawController.ClawStatus.OPEN) {
                        slides.currentStatus = SlidesController.SlidesStatus.AUTO_RUNTO;
                        clawPosition.currentStatus= ClawPositionController.ClawPositionStatus.SUB;
                        if(angle>0 && angle<=90)
                        {
                            clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.RUNTO;
                            claw_rotate_target = 0.003*angle + 0.52;
                        }
                        else if(angle>90 && angle<=180)
                        {
                            clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.RUNTO;
                            claw_rotate_target = 0.00322*angle - 0.0596;
                        }
                    }
                    double delay_collect = 1;
                    if(nr_slides>6)
                    {
                        delay_collect=1.3;
                    }
                    if(TimerSub.seconds()>delay_collect && claw.currentStatus!= ClawController.ClawStatus.CLOSE)
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerSub.seconds() > delay_collect+0.2) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                    }
                    if(TimerSub.seconds()>delay_collect+0.4)
                    {
                        fourbar.currentStatus= FourbarController.FourbarStatus.SUB;
                    }
                    if(TimerSub.seconds()>delay_collect+0.5)
                    {
                        clawPosition.currentStatus= ClawPositionController.ClawPositionStatus.BACKWARDS;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                    }
                    if(TimerSub.seconds()>delay_collect+0.5 && !follower.isBusy()) {
                        final Point scorePoint = new Point(follower.getPose().getX(), 103); //112
                        final Point scorePoint2 = new Point(34, 112);
                        score1 = new Path(new BezierCurve(new Point(follower.getPose())/*, scorePoint, scorePoint2*/, new Point(interPose)));
//                        score1.setTangentHeadingInterpolation();
                        score1.setLinearHeadingInterpolation(follower.getPose().getHeading(),interPose.getHeading());
//                        score1.setReversed(true);
                        follower.followPath(score1);
                    }
                    if(follower.getPose().getX()<40)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.BSK_HIGH_AUTO;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                        clawPosition.currentStatus= ClawPositionController.ClawPositionStatus.BACKWARDS;
                    }
                    if(follower.getPose().getX()<35)
                    {
                        follower.holdPoint(scoreSubPose);
                        TimerScore.reset();
                        status=SCORE_SUB;
                    }
                    break;
                }
                case SCORE_SUB:
                {
                    if(TimerScore.seconds() >0.35)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE_AUTO;
//                        clawRotate.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                    }
                    if(TimerScore.seconds() > 0.5)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerScore.seconds() > 0.6)
                    {
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;
                        cycle++;
                        detectedPoints.clear();
                        status= SUB_INTER;
                    }
                    break;
                }
            }
            linkage.update(linkage_current_position,linkage_target_position);
            slides.update(slides_current_position,slides_target_position);
            claw.update();
            fourbar.update();
            clawPosition.update();
            clawRotate.update(claw_rotate_target, 0);
            scoreSystem.update(robot.encoderClaw.getVoltage());
            follower.update();
            linkageSlides.update();
            telemetry.addData("status", status);
            telemetry.addData("cycle", cycle);
            if(!detectedPoints.isEmpty()) {
                telemetry.addData("detection", detectedPoints.get(0).point.toString());
            }
            telemetry.addData("nr_slides", nr_slides);
            telemetry.addData("slidescurrent", slides_current_position);
            telemetry.addData("linkagecurrent", linkage_current_position);
            telemetry.addData("current cycle x", Sx[cycle]);
            telemetry.addData("linkage_slides", linkageSlides.currentStatus);
            telemetry.addData("slidesstatus",slides.currentStatus);
            telemetry.addData("scoresystem",scoreSystem.currentStatus);
            telemetry.addData("Current Traj", follower.getCurrentPath());
            telemetry.addData("beam",beam.getState());
            telemetry.addData("ok", ok);
            telemetry.addData("pose", follower.getPose());
            Drawing.drawRobot(follower.getPose(),"#4CAF50");
            telemetry.update();
        }
        webcam.stopStreaming();
    }

}