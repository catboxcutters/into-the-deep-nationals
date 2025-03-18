package drive.writtenCode.controllers;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.spec.EllipticCurve;

import drive.writtenCode.RobotMap;

public class ScoreSystemController
{
    public enum ScoreSystemStatus {
        INIT,
        INIT_AUTO,
        LOWER_FOURBAR,
        GRAB_SAMPLE,
        LOWER_FOURBAR_SUB, COLLECT_FROM_SUB,BACKWARDS,BACKWARDS_SCORE,SAMPLE
        ,FLICK,SCORE, OPEN_RUNG, LOWER_FOURBAR_SUB_FEED,LOWER_FOURBAR_SUB_AUTO_ALIGN, RUNG, FEED, LOWER_FOURBAR_SUB_AUTO_ALIGN_RUNG, SCORE_LOW, COLLECT_FROM_SUB_INTER, START_COLLECT_FROM_SUB, COLLECT_SAMPLE, SCORE_AUTO, RUNG_PARA
    }
    public ScoreSystemStatus currentStatus = ScoreSystemStatus.INIT;
    public ScoreSystemStatus previousStatus=null;
    private LinkageController linkageController = null;
    private ClawController clawController = null;
    private ClawRotateController clawRotateController = null;
    private FourbarController fourbarController = null;
    private ClawPositionController clawPositionController = null;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime timer_sample = new ElapsedTime();
    public ElapsedTime timer_flick = new ElapsedTime();
    public static ElapsedTime timer_feed = new ElapsedTime();
    public ElapsedTime timer_reset = new ElapsedTime();
    public ElapsedTime timer_sub = new ElapsedTime();
    public double encoder_claw_position;
    double delay=0.21;
    double delay2=0.3;
    boolean ok;
    public ScoreSystemController(ClawController clawController, ClawRotateController clawRotateController, FourbarController fourbarController, ClawPositionController clawPositionController, LinkageController linkageController)
    {
        this.clawController = clawController;
        this.clawRotateController = clawRotateController;
        this.fourbarController = fourbarController;
        this.clawPositionController = clawPositionController;
        this.linkageController = linkageController;
    }
    public void update(double encoder_claw_position)
    {
        if(currentStatus!=previousStatus ||
                currentStatus==ScoreSystemStatus.GRAB_SAMPLE ||
                currentStatus==ScoreSystemStatus.LOWER_FOURBAR_SUB ||
                currentStatus==ScoreSystemStatus.LOWER_FOURBAR_SUB_FEED ||
                currentStatus==ScoreSystemStatus.FLICK ||
                currentStatus==ScoreSystemStatus.SCORE ||
                currentStatus==ScoreSystemStatus.SCORE_AUTO ||
                currentStatus==ScoreSystemStatus.RUNG ||
                currentStatus==ScoreSystemStatus.OPEN_RUNG ||
                currentStatus==ScoreSystemStatus.COLLECT_FROM_SUB_INTER ||
                currentStatus==ScoreSystemStatus.LOWER_FOURBAR_SUB_AUTO_ALIGN ||
                currentStatus==ScoreSystemStatus.LOWER_FOURBAR_SUB_AUTO_ALIGN_RUNG ||
                currentStatus==ScoreSystemStatus.FEED||
                currentStatus==ScoreSystemStatus.SAMPLE||
                currentStatus==ScoreSystemStatus.COLLECT_SAMPLE
        )
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                    fourbarController.currentStatus = FourbarController.FourbarStatus.INIT;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                    break;
                }
                case INIT_AUTO:
                {
                    clawController.currentStatus= ClawController.ClawStatus.CLOSE;
                    fourbarController.currentStatus = FourbarController.FourbarStatus.BACKWARDS;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.BACKWARDS;
                    break;
                }
                case LOWER_FOURBAR:
                {
                    fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT;
                    timer.reset();
                    currentStatus= ScoreSystemStatus.GRAB_SAMPLE;
                    ok=false;
                    break;
                }
                case GRAB_SAMPLE:
                {
                    if(timer.seconds()>delay && ok==false)
                    {
                        clawController.currentStatus = ClawController.ClawStatus.CLOSE_SPECIMEN;
                        timer.reset();
                        ok=true;
                    }
                    if(timer.seconds()>delay2 && ok==true)
                    {
                        currentStatus= ScoreSystemStatus.INIT;
                    }
                    break;
                }
                case START_COLLECT_FROM_SUB:
                {
                    timer_sub.reset();
                    currentStatus=ScoreSystemStatus.COLLECT_FROM_SUB_INTER;
                    break;
                }
                case COLLECT_FROM_SUB_INTER:
                {
                    if(timer_sub.seconds()<0.55) {
                        fourbarController.currentStatus = FourbarController.FourbarStatus.SUB_INTER;
                        clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.PERP;
                    }
                    else
                    {
                        currentStatus=ScoreSystemStatus.COLLECT_FROM_SUB;
                    }
                    break;
                }
                case COLLECT_FROM_SUB:
                {
                    fourbarController.currentStatus = FourbarController.FourbarStatus.SUB;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.SUB;
                    break;
                }
                case LOWER_FOURBAR_SUB:
                {
                        fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (timer.seconds() > 0.2) {
                            clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(timer.seconds()>0.3)
                        {
                            fourbarController.currentStatus= FourbarController.FourbarStatus.SUB;
                            clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.SUB;
                        }
                        if (timer.seconds() > 0.3 && encoder_claw_position>2.54) {
                            currentStatus = ScoreSystemStatus.INIT;
                        }
                        else if(timer.seconds()>0.45)
                        {
                            clawController.currentStatus= ClawController.ClawStatus.OPEN;
                            currentStatus = ScoreSystemStatus.COLLECT_FROM_SUB;
                        }
                    break;
                }
                case LOWER_FOURBAR_SUB_FEED:
                {
                    fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                    if (timer.seconds() > 0.2) {
                        clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                    }
                    if(timer.seconds()>0.3)
                    {
                        fourbarController.currentStatus= FourbarController.FourbarStatus.SUB;
                        clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.SUB;
                    }
                    if(timer.seconds()>0.45)
                    {
                        currentStatus = ScoreSystemStatus.COLLECT_FROM_SUB;
                    }
                    break;
                }
                case LOWER_FOURBAR_SUB_AUTO_ALIGN: {
                    if (timer.seconds() > 0.2) {
                        fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (timer.seconds() > 0.23) {
                            clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if (timer.seconds() > 0.4) {
                            currentStatus = ScoreSystemStatus.INIT;
                        }
                    }
                    break;
                }
                case LOWER_FOURBAR_SUB_AUTO_ALIGN_RUNG:
                {
                        fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (timer.seconds() > 0.2) {
                            clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(encoder_claw_position<2.55 && timer.seconds()>0.6 && timer.seconds()<0.7 && linkageController.currentStatus== LinkageController.LinkageStatus.COLLECT)
                        {
                                currentStatus=ScoreSystemStatus.COLLECT_FROM_SUB;
                                clawController.currentStatus= ClawController.ClawStatus.OPEN;
                        }
                        //aici am schimbat eu timerele putin
                        if (timer.seconds() > 0.5) {
                            fourbarController.currentStatus= FourbarController.FourbarStatus.RUNG_SIDE_RETRACT;
                            clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.RUNG_SIDE_RETRACT;
                            if(timer.seconds()>1.1)
                            {
                                linkageController.currentStatus= LinkageController.LinkageStatus.INIT;
                                clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.INIT;


                            }
                            if(timer.seconds()>1.5)
                            {
                                currentStatus = ScoreSystemStatus.INIT;
                            }
//                            else if(encoder_claw_position<2.59 && timer.seconds()>0.7 && linkageController.currentStatus== LinkageController.LinkageStatus.COLLECT)
//                            {
//                                currentStatus=ScoreSystemStatus.COLLECT_FROM_SUB;
//                                clawController.currentStatus= ClawController.ClawStatus.OPEN;
//                            }
                        }
                    break;
                }
                case FEED:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.FEED;
                    if(timer_feed.seconds()>0.1){
                        clawController.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(timer_feed.seconds()>0.5){
                        currentStatus=ScoreSystemStatus.INIT;
                    }
                    break;
                }
                case FLICK:
                {
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.FLICK;
                    if(timer_flick.seconds()>0.3)
                    {
                        clawController.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    break;
                }
                case SCORE:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.SCORE;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.SCORE;
                    timer_flick.reset();
                    break;
                }
                case SCORE_AUTO:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.SCORE_AUTO;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.SCORE_AUTO;
                    timer_flick.reset();
                    break;
                }
                case SCORE_LOW:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.SCORE_LOW;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.SCORE_LOW;
                    break;
                }
                case RUNG:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.RUNG;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.RUNG;
                    timer_reset.reset();
                    break;
                }
                case RUNG_PARA:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.RUNG_PARA;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.RUNG;
                    break;
                }
                case OPEN_RUNG:
                {
                    if(timer_reset.seconds()>0.18)
                    {
                        clawController.currentStatus = ClawController.ClawStatus.OPEN;
                    }
                    if(timer_reset.seconds()>0.5)
                    {
                        currentStatus=ScoreSystemStatus.INIT;
                    }
                    break;
                }
                case BACKWARDS:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.BACKWARDS;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.BACKWARDS;
                    break;
                }
                case BACKWARDS_SCORE:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.BACKWARDS_SCORE;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.BACKWARDS_SCORE;
                    break;
                }
                case SAMPLE:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.SAMPLE;
                    clawController.currentStatus= ClawController.ClawStatus.SAMPLE;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.SAMPLE;
                    timer_sample.reset();
                    break;
                }
                case COLLECT_SAMPLE:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.SAMPLE;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.COLLECT_SAMPLE;
                    if(timer_sample.seconds()>0.1)
                    {
                        clawController.currentStatus= ClawController.ClawStatus.CLOSE;
                    }
                    break;
                }
                }
            }
        }
    }

