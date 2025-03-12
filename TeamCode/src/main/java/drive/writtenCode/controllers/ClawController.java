package drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import drive.writtenCode.RobotMap;

@Config
public class  ClawController {
    public enum ClawStatus{
        OPEN,
        CLOSE,
        CLOSE_SPECIMEN,
        SAMPLE
    }
    public ClawStatus currentStatus = ClawStatus.OPEN;
    public ClawStatus previousStatus=null;

    public static double claw_open=0.55;
    public static double claw_closed = 0.77;
    public static double claw_closed_specimen = 0.78;
    public static double sample = 0.58;
    public Servo claw = null;


    public ClawController(RobotMap robot){
        this.claw = robot.clawOpen;
    }
    public void update(){
        if(currentStatus!=previousStatus)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case OPEN:
                {
                    this.claw.setPosition(claw_open);
                    break;
                }
                case CLOSE:
                {
                    this.claw.setPosition(claw_closed);
                    break;
                }
                case CLOSE_SPECIMEN:
                {
                    this.claw.setPosition(claw_closed_specimen);
                    break;
                }
                case SAMPLE:
                {
                    this.claw.setPosition(sample);
                    break;
                }
            }
        }
    }
}