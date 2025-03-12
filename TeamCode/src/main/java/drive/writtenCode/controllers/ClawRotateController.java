package drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import drive.writtenCode.RobotMap;

@Config
public class ClawRotateController {
    public enum ClawRotateStatus{
        INIT,
        MINUS,MINUS2,
        PLUS,PLUS2,
        AUTO_ALIGN,
        RUNTO, INVERSE,PICKUP1,PICKUP2,PICKUP3,;
    }
    public ClawRotateStatus currentStatus = ClawRotateStatus.INIT;
    public ClawRotateStatus previousStatus=null;
    public static double init_position=0.52;
    public static double inverse = 0;
    public Servo clawRotate = null;
    public ClawRotateController(RobotMap robot) {
        this.clawRotate=robot.clawRotate;
    }
    public void update(double target, double angle)
    {
        if(currentStatus!=previousStatus || currentStatus == ClawRotateStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.clawRotate.setPosition(init_position);
                    break;
                }
                case PLUS:
                {
                    this.clawRotate.setPosition(init_position+0.17);
                    break;
                }
                case PLUS2:
                {
                    this.clawRotate.setPosition(init_position+0.27);
                    break;
                }
                case MINUS:
                {
                    this.clawRotate.setPosition(init_position-0.17);
                    break;
                }
                case MINUS2:
                {
                    this.clawRotate.setPosition(init_position-0.29);
                    break;
                }
                case PICKUP1:
                {
                    this.clawRotate.setPosition(0.6);
                    break;
                }
                case PICKUP3:
                {
                    this.clawRotate.setPosition(0.43);
                    break;
                }
                case AUTO_ALIGN:
                {
                    if(0<=angle && angle<=22)
                    {
                        currentStatus = ClawRotateStatus.PLUS2;
                    }
                    else if(22<angle && angle<=67)
                    {
                        currentStatus = ClawRotateStatus.PLUS;
                    }
                    else if(67<angle && angle<=112)
                    {
                        currentStatus = ClawRotateStatus.INIT;
                    }
                    else if(112<angle && angle<=157)
                    {

                        currentStatus=ClawRotateStatus.MINUS;
                    }
                    else if(157<angle && angle<=180)
                    {
                        currentStatus=ClawRotateStatus.MINUS2;
                    }
                    break;
                }
                case INVERSE:
                {
                    this.clawRotate.setPosition(inverse);
                    break;
                }
                case RUNTO:
                {
                    this.clawRotate.setPosition(target);
                    break;
                }
            }
        }
    }
}