package drive.writtenCode.controllers;

import static drive.writtenCode.controllers.LinkageSlidesController.LinkageSlidesStatus.CLIMB;
import static drive.writtenCode.controllers.LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES;
import static drive.writtenCode.controllers.LinkageSlidesController.LinkageSlidesStatus.INIT;
import static drive.writtenCode.controllers.LinkageSlidesController.LinkageSlidesStatus.INIT_INTER;
import static drive.writtenCode.controllers.LinkageSlidesController.LinkageSlidesStatus.INIT_INTER_RUNG;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LinkageSlidesController
{
    public enum LinkageSlidesStatus {
        INIT,
        COLLECT,
        SCORE,
        LOWER_LINKAGE,
        EXTEND_SLIDES,
        INIT_INTER,
        BSK_HIGH,
        BSK_HIGH_AUTO,
        BSK_LOW,
        LOW_RUNG,
        LOW_RUNG_SCORE,
        KILL,
        CLIMB,
        HIGH_RUNG,
        EXTEND_SLIDES_RUNG, INIT_INTER_RUNG, LOWER_LINKAGE_RUNG, HIGH_RUNG_SCORE, HIGH_RUNG_AUTO
    }
    public LinkageSlidesStatus currentStatus = LinkageSlidesStatus.INIT;
    public LinkageSlidesStatus previousStatus=null;
    private SlidesController slidesController = null;
    private LinkageController linkageController = null;
    public static ElapsedTime timer = new ElapsedTime();
    public ElapsedTime timer_inter = new ElapsedTime();
    double delay=0.35;
    double delay2=0.3;
    double delay_rung_side = 1;
    boolean ok;
    public LinkageSlidesController(LinkageController linkageController, SlidesController slidesController)
    {
        this.linkageController = linkageController;
        this.slidesController = slidesController;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus==EXTEND_SLIDES || currentStatus==INIT_INTER || currentStatus==INIT_INTER_RUNG ||currentStatus==CLIMB)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                   linkageController.currentStatus = LinkageController.LinkageStatus.INIT;
                   slidesController.currentStatus = SlidesController.SlidesStatus.INIT;
                    break;
                }
                case LOWER_LINKAGE:
                {
                    linkageController.currentStatus = LinkageController.LinkageStatus.COLLECT;
                    timer.reset();
                    currentStatus = EXTEND_SLIDES;
                    break;
                }
                case EXTEND_SLIDES:
                {
                    if(timer.seconds()>delay)
                    {
                        if(slidesController.currentStatus != SlidesController.SlidesStatus.RUNTO_H && slidesController.currentStatus!= SlidesController.SlidesStatus.RUNTO) {
                            slidesController.currentStatus = SlidesController.SlidesStatus.COLLECT;
                        }
                        else {
                            slidesController.currentStatus = SlidesController.SlidesStatus.RUNTO_H;
                        }
                    }
                    break;
                }
                case CLIMB:
                {
                    slidesController.currentStatus=SlidesController.SlidesStatus.CLIMB;
                    if(timer.seconds()>0.5) {
                        linkageController.currentStatus = LinkageController.LinkageStatus.CLIMB;
                    }
                    break;
                }
                case INIT_INTER:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.INIT;
                    if(timer_inter.seconds()>0.4)
                    {
                        currentStatus=INIT;
                    }
                    break;
                }
                case INIT_INTER_RUNG:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.AUTO;
                    if(timer_inter.seconds()>0.55)
                    {
                        currentStatus=INIT;
                    }
                    break;
                }
                case BSK_HIGH:
                {
                    slidesController.currentStatus = SlidesController.SlidesStatus.BSK_HIGH;
                    linkageController.currentStatus= LinkageController.LinkageStatus.SCORE_HIGH;
                    break;
                }
                case BSK_HIGH_AUTO:
                {
                    slidesController.currentStatus = SlidesController.SlidesStatus.BSK_HIGH_AUTO;
                    linkageController.currentStatus= LinkageController.LinkageStatus.SCORE_HIGH;
                    break;
                }
                case BSK_LOW:
                {
                    slidesController.currentStatus = SlidesController.SlidesStatus.BSK_LOW;
                    linkageController.currentStatus= LinkageController.LinkageStatus.INIT;
                    break;
                }
                case LOW_RUNG:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.LOW_RUNG;
                    linkageController.currentStatus= LinkageController.LinkageStatus.INIT;
                    break;
                }
                case LOW_RUNG_SCORE:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.LOW_RUNG_SCORE;
                    linkageController.currentStatus= LinkageController.LinkageStatus.INIT;
                    break;
                }
                case HIGH_RUNG:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.HIGH_RUNG;
                    linkageController.currentStatus= LinkageController.LinkageStatus.SCORE_HIGH;
                    break;
                }
                case HIGH_RUNG_AUTO:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.HIGH_RUNG_AUTO;
                    linkageController.currentStatus= LinkageController.LinkageStatus.SCORE_HIGH;
                    break;
                }
                case HIGH_RUNG_SCORE:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.HIGH_RUNG_SCORE;
                    linkageController.currentStatus= LinkageController.LinkageStatus.SCORE_HIGH;
                    break;
                }
                case KILL:
                {
                    slidesController.currentStatus= SlidesController.SlidesStatus.KILL;
                    linkageController.currentStatus= LinkageController.LinkageStatus.KILL;
                    break;
                }
                //                case EXTEND_SLIDES_RUNG:
//                {
//                    if(timer.seconds()>delay_rung_side)
//                    {
//                        if(slidesController.currentStatus != SlidesController.SlidesStatus.RUNTO_H && slidesController.currentStatus!= SlidesController.SlidesStatus.RUNTO) {
//                            slidesController.currentStatus = SlidesController.SlidesStatus.COLLECT;
//                        }
//                        else {
//                            slidesController.currentStatus = SlidesController.SlidesStatus.RUNTO_H;
//                        }
//                    }
//                    break;
//                }
                //                case LOWER_LINKAGE_RUNG:
//                {
//                    linkageController.currentStatus = LinkageController.LinkageStatus.COLLECT;
//                    timer.reset();
//                    currentStatus = EXTEND_SLIDES_RUNG;
//                    break;
//                }
            }
        }
    }
}
