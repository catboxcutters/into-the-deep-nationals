package drive.writtenCode.auto;


import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.BASKET;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.BASKET_INTER;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.BASKET_SCORE;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.FAILSAFE_BACK;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.FAILSAFE_FORTH;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.GRAB_PICKUP1;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.GRAB_SPECIMEN1;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.GRAB_SPECIMEN2;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.GRAB_SPECIMEN3;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.GRAB_SPECIMEN4;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.PICKUP2;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.PICKUP3;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SCORE_SPECIMEN1;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SCORE_SPECIMEN2;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SCORE_SPECIMEN3;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SCORE_SPECIMEN4;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SPECIMEN1;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SPECIMEN2;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SPECIMEN3;
import static drive.writtenCode.auto.AutoSpecimenPush.STROBOT.SPECIMEN4;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import drive.writtenCode.RobotMap;
import drive.writtenCode.controllers.ClawController;
import drive.writtenCode.controllers.ClawPositionController;
import drive.writtenCode.controllers.ClawRotateController;
import drive.writtenCode.controllers.FourbarController;
import drive.writtenCode.controllers.LinkageController;
import drive.writtenCode.controllers.LinkageSlidesController;
import drive.writtenCode.controllers.ScoreSystemController;
import drive.writtenCode.controllers.SlidesController;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(group = "Auto")

public class AutoSpecimenPush extends LinearOpMode {


    public static double sweeper_active = 0.41, sweeper_inactive = 0.74;
    private void wait(int ms) {
        try{
            Thread.sleep(ms);
        }
        catch (InterruptedException ignored) {
            //nu face nimic
        }
    }


    public enum STROBOT
    {
        START,
        PLACE_PRELOAD,
        PICKUP1, GRAB_PICKUP1, DELIVER_PICKUP1, PICKUP2, DELIVER_PICKUP2, GRAB_PICKUP2, PICKUP3, GRAB_PICKUP3, DELIVER_PICKUP3, END_AUTO
        ,SPECIMEN1,GRAB_SPECIMEN1, GRAB_SPECIMEN2, SCORE_SPECIMEN2, SPECIMEN2, SPECIMEN3, GRAB_SPECIMEN3, SCORE_SPECIMEN3, SPECIMEN4, GRAB_SPECIMEN4, SCORE_SPECIMEN4, FAILSAFE, FAILSAFE_BACK, FAILSAFE_FORTH, BASKET, BASKET_INTER, BASKET_SCORE, SCORE_SPECIMEN1
    }
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerLeave = new ElapsedTime();
    ElapsedTime TimerPlacePreload = new ElapsedTime();
    ElapsedTime TimerPickup = new ElapsedTime();
    ElapsedTime TimerDeliver = new ElapsedTime();
    ElapsedTime TimerScore = new ElapsedTime();
    ElapsedTime TimerFailsafe = new ElapsedTime();
    private double time_place_preload = 1.2;
    private double time_leave_preload = 0.2;
    private Follower follower;

    int linkage_target_position = 0;
    int claw_rotate_target = 0;
    int slides_target_position = 0;

    private Path specimen1,specimen4,specimen4Score,specimen3,specimen3Score,specimen2,specimen2Score,specimen1Score,scorePreload,grabPickup1,deliver1,grabPickup2,deliver2,grabPickup3,deliver3,park,sample;
    private Path basketInter,basket,failsafe1_back, failsafe1_forth, failsafe2_back, failsafe2_forth, failsafe3_back, failsafe3_forth, failsafe4_back, failsafe4_forth;
    private PathChain pickup1_chain, pickup2_chain, pickup3_chain, basket_chain;
    public static double tunex=0;
    public static double tuney=0;
    final Pose startPose = new Pose(10, 63, Math.toRadians(0));
    final Pose preloadPose = new Pose(43,78, Math.toRadians(0)); //39.8
    final Pose pickup1Pose = new Pose(47, 31, Math.toRadians(0)); //52.7
    //    final Point pickup1Point = new Point(28, 47, Point.CARTESIAN);
    final Point pickup1Point = new Point(18, 32, Point.CARTESIAN);
    final Pose deliver1Pose = new Pose(21,25,Math.toRadians(0)); //32
    final Point deliver1Point = new Point(64,17, Point.CARTESIAN);

    final Pose pickup2Pose = new Pose(47,25,Math.toRadians(0)); //53
    final Pose deliver2Pose = new Pose(21,17,Math.toRadians(0));
    final Point deliver2Point = new Point(59, 8, Point.CARTESIAN);

    final Pose pickup3Pose = new Pose(47,14,Math.toRadians(0)); //53
    final Pose deliver3Pose = new Pose(21,15,Math.toRadians(0));
//    final Point deliver3Point = new Point(70, 2.6, Point.CARTESIAN);
    final Pose failsafe_pose1 = new Pose(22,15,Math.toRadians(0));
    final Pose specimen1Pose = new Pose(9,15,Math.toRadians(0)); //13.1 32.5
    //    final Point specimen1Point1 = new Point(31, 21, Point.CARTESIAN);
    final Point specimen1Point1 = new Point(34.3, 39, Point.CARTESIAN);
    final Point specimen1Point2 = new Point(28, 32.5, Point.CARTESIAN);


    final Pose specimen1ScorePose = new Pose(42.5 , 76, Math.toRadians(0));

    final Pose failsafe_pose2 = new Pose(22,34,Math.toRadians(0));
    final Pose specimen2Pose = new Pose(11,33.5,Math.toRadians(0)); //11.4
    final Point specimen2Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen2Point2 = new Point(51, 33.5, Point.CARTESIAN);
    final Pose specimen2ScorePose = new Pose(43, 74, Math.toRadians(0));

    final Pose failsafe_pose3 = new Pose(22,32.5,Math.toRadians(0));
    final Pose specimen3Pose = new Pose(11,33.5,Math.toRadians(0)); //11.4
    final Point specimen3Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen3Point2 = new Point(51, 33.5, Point.CARTESIAN);
    final Pose specimen3ScorePose = new Pose(43, 72, Math.toRadians(0));

    final Pose failsafe_pose4 = new Pose(22,32.5,Math.toRadians(0));
    final Pose specimen4Pose = new Pose(11,33.5,Math.toRadians(0));//11.4
    final Point specimen4Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen4Point2 = new Point(51, 33.5, Point.CARTESIAN);
    final Pose specimen4ScorePose = new Pose(43, 71, Math.toRadians(0));

    final Pose parkPose =new Pose(13,32.4, Math.toRadians(0));
    final Pose samplePose = new Pose(12,33.5,Math.toRadians(0));
    final Point samplePoint1 = new Point(28, 52, Point.CARTESIAN);
    final Point samplePoint2 = new Point(51, 33.5, Point.CARTESIAN);

    final Point basketPoint1 = new Point(15.514, 33.5, Point.CARTESIAN);
    final Point basketPoint2 = new Point(12.6,93.52);
    final Pose basketPoseInter = new Pose(14,119, Math.toRadians(90));
    final Pose basketPose = new Pose(10, 143, Math.toRadians(135));


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
        follower.setStartingPose(startPose);
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
        robot.sweep.setPosition(sweeper_inactive);
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
        scorePreload.setConstantHeadingInterpolation(preloadPose.getHeading());

        grabPickup1 = new Path(new BezierCurve(new Point(preloadPose), pickup1Point, new Point(pickup1Pose)));
        grabPickup1.setConstantHeadingInterpolation(pickup1Pose.getHeading());
//        grabPickup1.setZeroPowerAccelerationMultiplier(2.5);

        deliver1 = new Path(new BezierCurve(new Point(pickup1Pose) ,new Point(deliver1Pose)));
        deliver1.setConstantHeadingInterpolation(pickup1Pose.getHeading());

        pickup1_chain = follower.pathBuilder()
                .addPath(grabPickup1)
                .addPath(deliver1)
                .build();

        grabPickup2 = new Path(new BezierLine(new Point(deliver1Pose), new Point(pickup2Pose)));
        grabPickup2.setConstantHeadingInterpolation(pickup2Pose.getHeading());

        deliver2 = new Path(new BezierCurve(new Point(pickup2Pose), new Point(deliver2Pose)));
        deliver2.setConstantHeadingInterpolation(deliver2Pose.getHeading());

        pickup2_chain = follower.pathBuilder()
                .addPath(grabPickup2)
                .addPath(deliver2)
                .build();

        grabPickup3 = new Path(new BezierLine(new Point(deliver2Pose), new Point(pickup3Pose)));
        grabPickup3.setConstantHeadingInterpolation(pickup3Pose.getHeading());

        deliver3 = new Path(new BezierCurve(new Point(pickup3Pose), new Point(deliver3Pose)));
        deliver3.setConstantHeadingInterpolation(deliver3Pose.getHeading());

        pickup3_chain = follower.pathBuilder()
                .addPath(grabPickup3)
                .addPath(deliver3)
                .build();

        specimen1 = new Path(new BezierCurve(
                new Point(deliver3Pose),
                specimen1Point1,
                specimen1Point2,
                new Point(specimen1Pose)
        ));
        specimen1.setLinearHeadingInterpolation(deliver3Pose.getHeading(), specimen1Pose.getHeading());

        specimen1Score=new Path(new BezierLine(new Point(specimen1Pose), new Point(specimen1ScorePose)));
        specimen1Score.setLinearHeadingInterpolation(specimen1Pose.getHeading(), specimen1ScorePose.getHeading());

        specimen2 = new Path(new BezierCurve(
                new Point(specimen1ScorePose),
                specimen2Point1,
                specimen2Point2,
                new Point(specimen2Pose)
        ));
        specimen2.setLinearHeadingInterpolation(specimen1ScorePose.getHeading(), specimen2Pose.getHeading());
//        specimen2.setZeroPowerAccelerationMultiplier(2);

        specimen2Score=new Path(new BezierLine(new Point(specimen2Pose), new Point(specimen2ScorePose)));
        specimen2Score.setLinearHeadingInterpolation(specimen2Pose.getHeading(), specimen2ScorePose.getHeading());

        specimen3 = new Path(new BezierCurve(
                new Point(specimen2ScorePose),
                specimen3Point1,
                specimen3Point2,
                new Point(specimen3Pose)
        ));
        specimen3.setLinearHeadingInterpolation(specimen2ScorePose.getHeading(), specimen3Pose.getHeading());
//        specimen3.setZeroPowerAccelerationMultiplier(2);

        specimen3Score=new Path(new BezierLine(new Point(specimen3Pose), new Point(specimen3ScorePose)));
        specimen3Score.setLinearHeadingInterpolation(specimen3Pose.getHeading(), specimen3ScorePose.getHeading());

        specimen4 = new Path(new BezierCurve(
                new Point(specimen3ScorePose),
                specimen4Point1,
                specimen4Point2,
                new Point(specimen4Pose)
        ));
        specimen4.setLinearHeadingInterpolation(specimen3ScorePose.getHeading(), specimen4Pose.getHeading());
//        specimen4.setZeroPowerAccelerationMultiplier(2);

        specimen4Score=new Path(new BezierLine(new Point(specimen4Pose), new Point(specimen4ScorePose)));
        specimen4Score.setLinearHeadingInterpolation(specimen4Pose.getHeading(), specimen4ScorePose.getHeading());

        sample = new Path(new BezierCurve(new Point(specimen4ScorePose),samplePoint1,samplePoint2, new Point(samplePose)));
        sample.setLinearHeadingInterpolation(specimen4ScorePose.getHeading(),samplePose.getHeading());
//        basketInter = new Path(new BezierCurve(new Point(samplePose),basketPoint1,basketPoint2,new Point(basketPoseInter)));
//        basketInter.setTangentHeadingInterpolation();
        basketInter = new Path(new BezierCurve(new Point(samplePose),new Point(basketPoseInter)));
        basketInter.setLinearHeadingInterpolation(samplePose.getHeading(),basketPoseInter.getHeading());

        basket = new Path(new BezierLine(new Point(basketPoseInter), new Point(basketPose)));
        basket.setLinearHeadingInterpolation(Math.toRadians(90), basketPose.getHeading());

        basket_chain=new PathBuilder().addPath(basketInter).addPath(basket).build();

        failsafe1_back = new Path(new BezierLine(new Point(specimen1Pose), new Point(failsafe_pose1))); failsafe1_back.setConstantHeadingInterpolation(specimen1Pose.getHeading());
        failsafe1_forth = new Path(new BezierLine(new Point(failsafe_pose1), new Point(specimen1Pose))); failsafe1_forth.setConstantHeadingInterpolation(specimen1Pose.getHeading());
        failsafe2_back = new Path(new BezierLine(new Point(specimen2Pose), new Point(failsafe_pose2))); failsafe2_back.setConstantHeadingInterpolation(specimen2Pose.getHeading());
        failsafe2_forth = new Path(new BezierLine(new Point(failsafe_pose2), new Point(specimen2Pose))); failsafe2_forth.setConstantHeadingInterpolation(specimen2Pose.getHeading());
        failsafe3_back = new Path(new BezierLine(new Point(specimen3Pose), new Point(failsafe_pose3))); failsafe3_back.setConstantHeadingInterpolation(specimen3Pose.getHeading());
        failsafe3_forth = new Path(new BezierLine(new Point(failsafe_pose3), new Point(specimen3Pose))); failsafe3_forth.setConstantHeadingInterpolation(specimen3Pose.getHeading());
        failsafe4_back = new Path(new BezierLine(new Point(specimen4Pose), new Point(failsafe_pose4))); failsafe4_back.setConstantHeadingInterpolation(specimen4Pose.getHeading());
        failsafe4_forth = new Path(new BezierLine(new Point(failsafe_pose4), new Point(specimen4Pose))); failsafe4_forth.setConstantHeadingInterpolation(specimen4Pose.getHeading());

        park = new Path(new BezierLine(new Point(specimen4ScorePose),new Point(parkPose)));
        park.setLinearHeadingInterpolation(specimen4ScorePose.getHeading(), parkPose.getHeading());
        boolean ok=false;
        int current_specimen = 1;

        STROBOT status = STROBOT.START;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int slides_current_position = robot.linkage.getCurrentPosition();
            int linkage_current_position = linkage.encoderLinkage.getCurrentPosition();
            switch (status) {
                case START: {
                    AutoTimer.reset();
                    TimerPlacePreload.reset();
//                    follower.followPath(scorePreload,true);
                    follower.holdPoint(preloadPose);
                    scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
//                    linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                    status = STROBOT.PLACE_PRELOAD;
                    break;
                }
                case PLACE_PRELOAD: {
                    if(TimerPlacePreload.seconds()>0)
                    {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_AUTO;
                    }
                    if(TimerPlacePreload.seconds()>time_place_preload) {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                        status = STROBOT.PICKUP1;
                    }
                    break;
                }
                case PICKUP1:
                {
                    if(TimerLeave.seconds()>time_leave_preload)
                    {
//                        follower.followPath(grabPickup1, false);
                        follower.followPath(pickup1_chain, true);
                        TimerDeliver.reset();
                        status= GRAB_PICKUP1;
                    }
                    break;
                }
                case GRAB_PICKUP1:
                {
                    if(TimerLeave.seconds()>time_leave_preload+0.2)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        if(follower.getCurrentPath()==deliver1)
                        {
                            robot.sweep.setPosition(sweeper_active);
                        }
                        if(!follower.isBusy() || (follower.getPose().getX()<deliver1Pose.getX()+0.5 && TimerDeliver.seconds()>1.5))
                        {
                            robot.sweep.setPosition(sweeper_inactive);
                            TimerLeave.reset();
                            status= PICKUP2;
                        }
                    }

                    break;
                }
                case PICKUP2:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(pickup2_chain, true);
                        TimerDeliver.reset();
                        status=STROBOT.GRAB_PICKUP2;
                    }
                    break;
                }
                case GRAB_PICKUP2:
                {
                    if(follower.getCurrentPath()==deliver2)
                    {
                        robot.sweep.setPosition(sweeper_active);
                    }
                    if(!follower.isBusy() || (follower.getPose().getX()<deliver2Pose.getX()+0.5 && TimerDeliver.seconds()>1.7))
                    {
                        robot.sweep.setPosition(sweeper_inactive);
                        TimerLeave.reset();
                        status=PICKUP3;
                    }
                    break;
                }
                case PICKUP3:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(pickup3_chain, true);
                        TimerDeliver.reset();
                        status=STROBOT.GRAB_PICKUP3;
                    }
                    break;
                }
                case GRAB_PICKUP3:
                {
                    if(follower.getCurrentPath()==deliver3)
                    {
                        robot.sweep.setPosition(sweeper_active);
                        follower.setMaxPower(0.8);
                    }
                    if(follower.getPose().getX()<deliver3Pose.getX()+1.5 && TimerDeliver.seconds()>1)
                    {
                        TimerLeave.reset();
                        status=SPECIMEN1;
                    }
                    break;
                }
                case SPECIMEN1:
                {

                    if(TimerLeave.seconds()>0)
                    {
//                        follower.followPath(specimen1);
                        follower.holdPoint(specimen1Pose);
                        status = GRAB_SPECIMEN1;
                        current_specimen = 1;
                        TimerLeave.reset();
                    }
                    break;
                }
                case GRAB_SPECIMEN1:
                {
                    if(TimerDeliver.seconds()>1.5)
                    {
                        robot.sweep.setPosition(sweeper_inactive);
                    }
                    if(beam.getState()==false && TimerLeave.seconds()>0.5)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            TimerLeave.reset();
                            ok=true;
                        }
                    }
                    else if(beam.getState()==true && TimerLeave.seconds()>2.3 && claw.currentStatus == ClawController.ClawStatus.OPEN)
                    {
                        status=FAILSAFE_BACK;
                    }
                    if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>0.2)
                    {
                        status = SCORE_SPECIMEN1;
                        ok=false;
                        follower.setMaxPower(1);
                        follower.holdPoint(specimen1ScorePose);
//                            follower.followPath(specimen1Score);
                        TimerScore.reset();
                    }
                    break;
                }
                case SCORE_SPECIMEN1:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if((TimerScore.seconds()>1.5 && follower.getPose().getX()>specimen1ScorePose.getX()-1 && linkageSlides.currentStatus == LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    || TimerScore.seconds()>1.7)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(TimerScore.seconds()>1.8
                            && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE
                            )
                    {
                        ok=false;
                        status=SPECIMEN2;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN2:
                {
                    follower.followPath(specimen2);
                    ok=false;
                    status = GRAB_SPECIMEN2;
                    current_specimen = 2;
                    TimerLeave.reset();
                    break;
                }
                case GRAB_SPECIMEN2:
                {
                    if(beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            TimerLeave.reset();
                            ok=true;
                        }
                    }
                    else if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>2.5 && claw.currentStatus == ClawController.ClawStatus.OPEN)
                    {
                        status=FAILSAFE_BACK;
                    }
                    if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>0.4)
                    {
                        status = SCORE_SPECIMEN2;
                        ok=false;
                        follower.holdPoint(specimen2ScorePose);
                        TimerScore.reset();
                    }
                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT) {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    break;
                }
                case SCORE_SPECIMEN2:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if((TimerScore.seconds()>1.5 && follower.getPose().getX()>specimen2ScorePose.getX()-1 && linkageSlides.currentStatus == LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                            || TimerScore.seconds()>1.7)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(TimerScore.seconds()>1.9
                            && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE
                    )
                    {
                        ok=false;
                        status=SPECIMEN3;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN3:
                {
                    follower.followPath(specimen3);
                    ok=false;
                    status = GRAB_SPECIMEN3;
                    current_specimen = 3;
                    TimerLeave.reset();
                    break;
                }
                case GRAB_SPECIMEN3:
                {

                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            TimerLeave.reset();
                            ok=true;
                        }
                    }
                    else if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>2.5 && claw.currentStatus == ClawController.ClawStatus.OPEN)
                    {
                        status=FAILSAFE_BACK;
                    }
                    if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>0.4)
                    {
                        status = SCORE_SPECIMEN3;
                        ok=false;
                        follower.holdPoint(specimen3ScorePose);
                        TimerScore.reset();
                    }
                    break;
                }
                case SCORE_SPECIMEN3:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if((TimerScore.seconds()>1.3 && follower.getPose().getX()>specimen3ScorePose.getX()-1 && linkageSlides.currentStatus == LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                            || TimerScore.seconds()>1.7)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(TimerScore.seconds()>1.5
                            && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE
                    )
                    {
                        ok=false;
                        status=SPECIMEN4;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN4:
                {
                        follower.followPath(specimen4);
                        ok = false;
                        status = GRAB_SPECIMEN4;
                        current_specimen = 4;
                        TimerLeave.reset();
                    break;
                }
                case GRAB_SPECIMEN4:
                {
                    if(TimerLeave.seconds()>0.2 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            TimerLeave.reset();
                            ok=true;
                        }
                    }
                    if(!follower.isBusy() && beam.getState()==true && TimerLeave.seconds()>2)
                    {
//                        status=FAILSAFE_BACK;
                    }
                    if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>0.4)
                    {
                        status = SCORE_SPECIMEN4;
                        ok=false;
                        follower.holdPoint(specimen4ScorePose);
                        TimerScore.reset();
                    }
                    break;
                }
                case SCORE_SPECIMEN4:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if((TimerScore.seconds()>1 && follower.getPose().getX()>specimen4ScorePose.getX()-1 && linkageSlides.currentStatus == LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                            || TimerScore.seconds()>2)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                    }
                    if(follower.getPose().getX() < 39   && linkageSlides.currentStatus == LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE && TimerScore.seconds()>1.5)
                    {
                        ok=false;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.INIT;
                        status=BASKET_INTER;
                    }
                    else if(!follower.isBusy() && TimerScore.seconds()>1.7  && linkageSlides.currentStatus == LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE)
                    {
                        TimerLeave.reset();
                        fourbar.currentStatus= FourbarController.FourbarStatus.COLLECT_SAMPLE;
                        follower.followPath(sample);
                    }
                    break;
                }
                case BASKET_INTER:
                {
                    if(TimerLeave.seconds()>0.5)
                    {
                        follower.setMaxPower(0.8);
                    }
                    if(beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            TimerLeave.reset();
                            ok=true;
                        }
                    }
                    if(claw.currentStatus == ClawController.ClawStatus.CLOSE_SPECIMEN && TimerLeave.seconds()>0.4)
                    {
                        status = BASKET;
                        ok=false;
                        follower.setMaxPower(1);
                        follower.followPath(basket_chain);
                        TimerScore.reset();
                    }
                    break;
                }
                case BASKET:
                {
                    if(TimerScore.seconds()>1)
                    {
                        fourbar.currentStatus= FourbarController.FourbarStatus.BACKWARDS;
                    }
                    if(TimerScore.seconds()>2)
                    {
//                        follower.holdPoint(basketPose);
                        slides.currentStatus= SlidesController.SlidesStatus.BSK_HIGH;
                        status=BASKET_SCORE;
                    }
                    break;
                }
                case BASKET_SCORE:
                {
                    if(TimerScore.seconds()>2.7)
                    {
                        fourbar.currentStatus= FourbarController.FourbarStatus.RUNG;
                        clawPosition.currentStatus= ClawPositionController.ClawPositionStatus.RUNG;
                    }
                    if(TimerScore.seconds()>2.9)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerScore.seconds()>3.1)
                    {
                        fourbar.currentStatus= FourbarController.FourbarStatus.INIT;
                        clawPosition.currentStatus= ClawPositionController.ClawPositionStatus.INIT;
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    }
                    break;
                }
                case FAILSAFE_BACK:
                {
                    switch (current_specimen)
                    {
                        case 1:
                        {
                            follower.followPath(failsafe1_back);
                            TimerLeave.reset();
                            status = FAILSAFE_FORTH;
                            break;
                        }
                        case 2:
                        {
                            follower.followPath(failsafe2_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                        case 3:
                        {
                            follower.followPath(failsafe3_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                        case 4:
                        {
                            follower.followPath(failsafe4_back);
                            status = FAILSAFE_FORTH;
                            break;
                        }
                    }
                    break;
                }
                case FAILSAFE_FORTH:
                {
                    if(!follower.isBusy())
                    {
                        switch(current_specimen)
                        {
                            case 1:
                            {
                                follower.followPath(failsafe1_forth);
                                status = GRAB_SPECIMEN1;
                                break;
                            }
                            case 2:
                            {
                                follower.followPath(failsafe2_forth);
                                status = GRAB_SPECIMEN2;
                                break;
                            }
                            case 3:
                            {
                                follower.followPath(failsafe3_forth);
                                status = GRAB_SPECIMEN3;
                                break;
                            }
                            case 4:
                            {
                                follower.followPath(failsafe4_forth);
                                status = GRAB_SPECIMEN4;
                                break;
                            }
                        }
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
            telemetry.addData("follower busy", follower.isBusy());
            telemetry.addData("slidescurrent", slides_current_position);
            telemetry.addData("clawposition",clawPosition.currentStatus);
            telemetry.addData("scoresysten",scoreSystem.currentStatus);
            telemetry.addData("Current Traj", follower.getCurrentPath());
            telemetry.addData("beam",beam.getState());
            telemetry.addData("ok", ok);
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }
    }

}