package drive.writtenCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import drive.writtenCode.controllers.ClawController;
import drive.writtenCode.controllers.ClawPositionController;
import drive.writtenCode.controllers.ClawRotateController;
import drive.writtenCode.controllers.ClimbController;
import drive.writtenCode.controllers.FourbarController;
import drive.writtenCode.controllers.LinkageController;
import drive.writtenCode.controllers.LinkageSlidesController;
import drive.writtenCode.controllers.PTOController;
import drive.writtenCode.controllers.ScoreSystemController;
import drive.writtenCode.controllers.SlidesController;

@Config
@TeleOp(name="TeleOpCode", group="Linear OpMode")
public class TeleOpCode extends LinearOpMode {
    public void setMotorRunningMode(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode) {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }

    public void setMotorZeroPowerBehaviour(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack,
                                  DcMotor rightFront, DcMotor rightBack,
                                  double leftTrigger, double rightTrigger, double rate,double rot_rate, double strafe_brake) {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = (-gamepad1.left_trigger + gamepad1.right_trigger) * 1.05 * strafe_brake; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double leftFrontPower = (y + x + rx*rot_rate) / denominator *rate;
        double leftBackPower = (y - x + rx*rot_rate) / denominator *rate;
        double rightFrontPower = (y - x - rx*rot_rate) / denominator *rate;
        double rightBackPower = (y + x - rx*rot_rate) / denominator *rate;


        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    ElapsedTime GlobalTimer = new ElapsedTime();
    ElapsedTime BeamTimer = new ElapsedTime();
    ElapsedTime SlidesTimer = new ElapsedTime();
    ElapsedTime timer_release = new ElapsedTime();
    ElapsedTime climb_timer = new ElapsedTime();
    ElapsedTime stable_timer = new ElapsedTime();
    public static boolean ok_climb_stable = false;
    public static boolean ok_climb = false;
    public static double test= 0.5;

    RevBlinkinLedDriver.BlinkinPattern off= RevBlinkinLedDriver.BlinkinPattern.BLACK;
    RevBlinkinLedDriver.BlinkinPattern on= RevBlinkinLedDriver.BlinkinPattern.GREEN;
    double encoder_position;
    double time_to_slides = 0.5;
    double time_to_start_beam = 0.05;
    int linkage_target_position = 0;
    int slides_target_position = 0;
    float strafe_brake = 1;
    double claw_rotate_target =0;
    boolean use_dist = false;
    boolean use_encoder = true;
    public double rate=1;
    public double rot_rate=1;
    boolean ok=false;
    boolean ok_rung = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotMap robot= new RobotMap(hardwareMap);

        LinkageController linkageController = new LinkageController(robot);
        SlidesController slidesController = new SlidesController(robot);
        FourbarController fourbarController = new FourbarController(robot);
        ClawRotateController clawRotateController = new ClawRotateController(robot);
        ClawController clawController = new ClawController(robot);
        ClawPositionController clawPositionController = new ClawPositionController(robot);
        ScoreSystemController scoreSystemController = new ScoreSystemController(clawController, clawRotateController,fourbarController,clawPositionController, linkageController);
        LinkageSlidesController linkageSlidesController = new LinkageSlidesController(linkageController, slidesController);
        ClimbController climbController = new ClimbController(robot);
        DigitalChannel beam = robot.beam;
        PTOController ptoController = new PTOController(robot);
//        linkageController.update(LinkageController.init_position,linkage_target_position);
//        slidesController.update(SlidesController.init_position,slides_target_position);
        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT_AUTO;


        scoreSystemController.update(robot.encoderClaw.getVoltage());
        clawRotateController.update(ClawRotateController.init_position,0);
        fourbarController.update();
        clawPositionController.update();
        clawController.update();
        linkageSlidesController.update();
        climbController.update();
        ptoController.update();

        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        MotorConfigurationType mct1, mct2, mct3, mct4;
        mct1 = rightBack.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBack.setMotorType(mct1);

        mct2 = rightFront.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFront.setMotorType(mct2);

        mct3 = leftFront.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFront.setMotorType(mct3);

        mct4 = leftBack.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBack.setMotorType(mct4);
        setMotorRunningMode(leftFront,leftBack,rightFront,rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        setMotorZeroPowerBehaviour(leftFront,leftBack,rightFront,rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        robot.sweep.setPosition(0.74);
//        cameraController.startCamera();
        ptoController.currentStatus= PTOController.PTOStatus.INACTIVE;
        climb_timer.reset();
        waitForStart();
        climb_timer.reset();
        GlobalTimer.reset();
        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
        clawController.currentStatus= ClawController.ClawStatus.OPEN;
        while(opModeIsActive())
        {
            if(isStopRequested()) return;
            int slides_current_position = robot.encoderSlides.getCurrentPosition();
            int linkage_current_position = robot.encoderLinkage.getCurrentPosition();
            if(ptoController.currentStatus!= PTOController.PTOStatus.ACTIVE) {
                robotCentricDrive(leftFront, leftBack, rightFront, rightBack, gamepad1.left_trigger, gamepad1.right_trigger, rate, rot_rate, strafe_brake);
            }
            else
            {
                leftBack.setPower(0);
                rightBack.setPower(0);
                robotCentricDrive(leftFront, leftFront, rightFront, rightFront, gamepad1.left_trigger, gamepad1.right_trigger, rate, rot_rate, strafe_brake);
            }

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.y && !previousGamepad1.y)
            {
                switch(linkageSlidesController.currentStatus) {
                    default:
                        LinkageSlidesController.timer.reset();
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.CLIMB;
                        break;
                    case CLIMB:
                        robot.linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        robot.slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        robot.slidesMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        robot.slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.KILL;
                        ptoController.currentStatus = PTOController.PTOStatus.ACTIVE;
                        break;
                }
            }
            if(ok_climb_stable==false)
            {
                stable_timer.reset();
            }
            if(ok_climb==false)
            {
                climb_timer.reset();
            }
            if(climb_timer.seconds()>0.6 && linkageController.currentStatus!= LinkageController.LinkageStatus.STABLE)
            {
                linkageController.currentStatus= LinkageController.LinkageStatus.INIT;
                robot.linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (slidesController.currentStatus== SlidesController.SlidesStatus.PTO || (slidesController.currentStatus == SlidesController.SlidesStatus.KILL && ptoController.currentStatus== PTOController.PTOStatus.ACTIVE))
            {
                if(gamepad1.left_stick_y<0 || gamepad1.left_stick_y<0)
                {
                    slidesController.currentStatus=SlidesController.SlidesStatus.PTO;
                    ok_climb=true;
                }
                else
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.KILL;
                }
            }
            if(currentGamepad2.dpad_right)
            {
                climbController.currentStatus = ClimbController.ClimbStatus.DOWN;
            }
            else
            {
                climbController.currentStatus = ClimbController.ClimbStatus.STOP;
            }
            if(currentGamepad2.dpad_left)
            {
                climbController.currentStatus= ClimbController.ClimbStatus.UP;
            }
            if(currentGamepad2.dpad_left)
            {
                climbController.currentStatus = ClimbController.ClimbStatus.UP;
            }
//            if(currentGamepad1.dpad_down)
//            {
//                robot.linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                linkageController.currentStatus= LinkageController.LinkageStatus.STABLE;
//                ok_climb_stable=true;
//            }
//            if(stable_timer.seconds()>4 && stable_timer.seconds()<5)
//            {
////                robot.linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                linkageController.currentStatus= LinkageController.LinkageStatus.INIT;
//            }
//            else if(stable_timer.seconds()>5 && linkageController.currentStatus== LinkageController.LinkageStatus.INIT)
//            {
//                linkageController.currentStatus= LinkageController.LinkageStatus.KILL;
//                robot.linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            }
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            {
                switch(linkageSlidesController.currentStatus){
                    case INIT:
                        clawController.currentStatus = ClawController.ClawStatus.OPEN;
                        linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.LOWER_LINKAGE;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.START_COLLECT_FROM_SUB;
                        rate = 0.3;
                        rot_rate=1;
                        break;
                    case EXTEND_SLIDES:
                        scoreSystemController.timer.reset();
                        SlidesTimer.reset();
                        ok=true;
                        scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR_SUB;
                        break;
                }
            }
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            {
                switch(linkageSlidesController.currentStatus){
                    case INIT:
                        clawController.currentStatus = ClawController.ClawStatus.OPEN;
                        linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.LOWER_LINKAGE;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.START_COLLECT_FROM_SUB;
                        rate = 0.3;
                        rot_rate=1;
                        break;
                    case EXTEND_SLIDES:
                        scoreSystemController.timer.reset();
                        SlidesTimer.reset();
                        ok_rung = true;
                        scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR_SUB_AUTO_ALIGN_RUNG;
                        break;
                }
            }
            if(currentGamepad1.x && !previousGamepad1.x)
            {
                switch(linkageSlidesController.currentStatus){
                    case INIT:
                        clawController.currentStatus = ClawController.ClawStatus.OPEN;
                        linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.LOWER_LINKAGE;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.START_COLLECT_FROM_SUB;
                        rate = 0.3;
                        rot_rate=1;
                        break;
                    case EXTEND_SLIDES:
                        scoreSystemController.timer.reset();
                        ok=false;
                        scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR_SUB_FEED;
                        break;
                }
            }
            if(SlidesTimer.seconds()>time_to_slides && ok==true && clawController.currentStatus== ClawController.ClawStatus.CLOSE)
            {
                linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                time_to_slides = 0.5;
                ok=false;
            }
            if(SlidesTimer.seconds()>0.9 && ok_rung==true && clawController.currentStatus== ClawController.ClawStatus.CLOSE && scoreSystemController.currentStatus!= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB)
            {
                linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT_INTER_RUNG;
                linkageSlidesController.timer_inter.reset();
                time_to_slides = 0.5;
                ok_rung=false;
            }
            if(linkageSlidesController.currentStatus== LinkageSlidesController.LinkageSlidesStatus.INIT)
            {
                rate = 1;
                rot_rate=1;
            }
            if(currentGamepad2.y && !previousGamepad2.y)
            {
                switch(scoreSystemController.currentStatus){
                    case INIT:
                        scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                        break;
                }
            }
            if(currentGamepad2.a && !previousGamepad2.a)
            {
                switch(linkageSlidesController.currentStatus)
                {
                    case INIT:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.SCORE;
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS2;
                        break;
                    default:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                        clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                        clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.INIT;
                        break;
                }
            }
            if(currentGamepad2.x && !previousGamepad2.x)
            {
                switch(linkageSlidesController.currentStatus)
                {
                    case INIT:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_LOW;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.SCORE_LOW;
                        clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.PLUS2;
                        break;
                    default:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                        clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.INIT;
                        clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.INIT;
                        break;
                }
            }
            if(currentGamepad2.b && !previousGamepad2.b)
            {
                switch(linkageSlidesController.currentStatus)
                {
                    case INIT:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.RUNG;
                        break;
                    case HIGH_RUNG:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        timer_release.reset();
                        break;
                }
            }
            if(timer_release.seconds()>0.4 && linkageSlidesController.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE) {
                linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
            }
                if(beam.getState()==true)
            {
                BeamTimer.reset();
            }
            else if(BeamTimer.seconds()>time_to_start_beam && scoreSystemController.currentStatus == ScoreSystemController.ScoreSystemStatus.INIT)
            {
                scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                BeamTimer.reset();
            }

            boolean manual_gm2 = false;
            boolean manual_gm1 = false;
            boolean manual = false;
            if (gamepad2.left_stick_y <-0.4 || (gamepad1.right_stick_y<-0.4 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES) )
            {
                int manual_rate = 0;
                manual = true;
                if(gamepad1.right_stick_y<-0.4 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES && -robot.encoderLinkage.getCurrentPosition()>2300)
                {
                    manual_rate = 1500;
                    slidesController.currentStatus= SlidesController.SlidesStatus.RUNTO_H;
                    slides_target_position =
                            Math.max(-36000, Math.min(-slides_current_position-manual_rate,-17000));
                }
                else if(gamepad2.left_stick_y < -0.4 && linkageSlidesController.currentStatus != LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES)
                {
                    manual_rate = 4000;
                    slidesController.currentStatus = SlidesController.SlidesStatus.RUNTO;
                    slides_target_position =
                            Math.max(-66000, Math.min(-slides_current_position-manual_rate,-20000));
                    manual_gm2=true;
                }
            }
            else
            if (gamepad2.left_stick_y >0.4 || (gamepad1.right_stick_y>0.4 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES))
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
                int manual_rate = 0;
                manual = true;
                if(gamepad1.right_stick_y>0.4 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES && -robot.encoderLinkage.getCurrentPosition()>2300)
                {
                    manual_rate = 1500;
                    slidesController.currentStatus= SlidesController.SlidesStatus.RUNTO_H;
                    slides_target_position =
                            Math.max(- 36000, Math.min(-slides_current_position+manual_rate,-17000));
                }
                else if(gamepad2.left_stick_y > 0.4 && linkageSlidesController.currentStatus != LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES)
                {
                    manual_rate = 4000;
                    slidesController.currentStatus = SlidesController.SlidesStatus.RUNTO;
                    slides_target_position =
                            Math.max(- 63500, Math.min(-slides_current_position+manual_rate,60000));
                    manual_gm2=true;
                }
                  //Math.max(slides_current_position+10,//era +10, dar mergea invers 0)
            }
            else if(gamepad1.right_stick_button && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES)
            {
                int manual_rate = 4000;
                slidesController.currentStatus= SlidesController.SlidesStatus.RUNTO_H;
                slides_target_position =
                        Math.max(-36000, Math.min(-slides_current_position-manual_rate,-17000));
            }
            else
            {
                slides_target_position = -slides_current_position;
            }
            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            {
                switch(clawRotateController.currentStatus)
                {
                    case INIT:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS;
                        break;
                    case MINUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS2;
                        break;
                    case PLUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                        break;
                    case PLUS2:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS;
                        break;
                }
            }
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper)
            {
                switch(clawRotateController.currentStatus)
                {
                    case INIT:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS;
                        break;
                    case PLUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS2;
                        break;
                    case MINUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                        break;
                    case MINUS2:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS;
                        break;
                }
            }

            //asta trb sa fie mereu la final
            if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down)
            {
               switch (linkageSlidesController.currentStatus) {
                   default:
                       switch (clawController.currentStatus) {
                        case CLOSE:
                        case CLOSE_SPECIMEN:
                               clawController.currentStatus = ClawController.ClawStatus.OPEN;
                               break;
                        case OPEN:
                               clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                               break;
                       }
                        break;
                   case SCORE:
                   case BSK_HIGH:
//                   case BSK_LOW:
                       switch(scoreSystemController.currentStatus)
                          {
                              case SCORE:
//                                 scoreSystemController.timer_flick.reset();
//                                 scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.FLICK;
                                  switch(clawController.currentStatus) {
                                      case CLOSE:
                                          clawController.currentStatus= ClawController.ClawStatus.OPEN;
                                          break;
                                      case CLOSE_SPECIMEN:
                                          clawController.currentStatus= ClawController.ClawStatus.OPEN;
                                          break;
                                      case OPEN:
                                          clawController.currentStatus= ClawController.ClawStatus.CLOSE;
                                          break;
                                  }
                                  break;
                                case FLICK:
                                 scoreSystemController.timer_flick.reset();
                                 scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.SCORE;
                                 break;
                          }
               }
            }

            if(currentGamepad2.right_trigger>0.2 && previousGamepad2.right_trigger<0.2)
            {
                switch(fourbarController.currentStatus)
                {
                    case RUNG:
                    {
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.RUNG_PARA;
                        break;
                    }
                    case RUNG_PARA:
                    {
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.RUNG;
                        break;
                    }
                }
            }
            if(-slides_current_position > SlidesController.collect_position
                    && -slides_current_position < SlidesController.collect_position+8000
                    && linkageSlidesController.currentStatus== LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES
                    && linkageSlidesController.timer.seconds()<0.8)
            {
                slidesController.currentStatus= SlidesController.SlidesStatus.RUNTO_H;
                slides_target_position = -16000;
//                        Math.max(-22000, Math.min(slides_current_position-1500,-20000));
            }
            if(currentGamepad1.a && !previousGamepad1.a)
            {
                double offset=-0.04;
                ClawPositionController.collect_position = ClawPositionController.collect_position + offset;
                ClawPositionController.collect_sub_position = ClawPositionController.collect_sub_position + offset;
                ClawPositionController.perp_position = ClawPositionController.perp_position + offset;
                ClawPositionController.rung_position = ClawPositionController.rung_position + offset;
                ClawPositionController.init_position = ClawPositionController.init_position + offset;
                ClawPositionController.flick = ClawPositionController.flick + offset;
                ClawPositionController.rung_side_retract = ClawPositionController.rung_side_retract + offset;
                ClawPositionController.score = ClawPositionController.score + offset;
                ClawPositionController.score_low = ClawPositionController.score_low + offset;
                ClawPositionController.sub_position = ClawPositionController.sub_position + offset;

            }
            if(currentGamepad1.b && !previousGamepad1.b)
            {
                if(use_encoder==false)
                {
                    use_encoder=true;
                }
                else
                {
                    use_encoder=false;
                }
            }
            if(use_encoder==false)
            {
                encoder_position = 3;
            }
            else
            {
                encoder_position = robot.encoderClaw.getVoltage();
            }
            linkageController.update(linkage_current_position,linkage_target_position);
            slidesController.update(slides_current_position,slides_target_position);
            clawController.update();
            fourbarController.update();
            clawPositionController.update();
            clawRotateController.update(claw_rotate_target, 0);
            scoreSystemController.update(encoder_position);
            climbController.update();
            ptoController.update();
            linkageSlidesController.update();

            telemetry.addData("fourbar status", fourbarController.currentStatus);
            telemetry.addData("beam", beam.getState());
            telemetry.addData("score system", scoreSystemController.currentStatus);
            telemetry.addData("linkageslides system", linkageSlidesController.currentStatus);
            telemetry.addData("slides", slidesController.currentStatus);
            telemetry.addData("slides_max_speed", slidesController.PID.maxOutput);
            telemetry.addData("slides_target", slides_target_position);
            telemetry.addData("slides_current", slides_current_position);
            telemetry.addData("linkage_position", robot.encoderLinkage.getCurrentPosition());
            telemetry.addData("slides_PID_rung", slidesController.PID == slidesController.slidesPID_rung);
            telemetry.addData("encoder_claw", robot.encoderClaw.getVoltage());
            telemetry.update();
        }
    }
}