package drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import drive.writtenCode.RobotMap;

@Config
public class PTOController {
    public enum PTOStatus{
        ACTIVE,
        INACTIVE;
    }
    public PTOStatus currentStatus = PTOStatus.INACTIVE;
    public PTOStatus previousStatus=null;

    public static double PTO_active[]={0,1};
    public static double PTO_inactive[] = {0.58,0.5};
    public Servo PTOLeft = null, PTORight = null;


    public PTOController(RobotMap robot){
        this.PTOLeft = robot.PTOLeft;
        this.PTORight = robot.PTORight;
    }
    public void update(){
        if(currentStatus!=previousStatus)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INACTIVE:
                {
                    this.PTORight.setPosition(PTO_inactive[0]);
                    this.PTOLeft.setPosition(PTO_inactive[1]);
                    break;
                }
                case ACTIVE:
                {
                    this.PTORight.setPosition(PTO_active[0]);
                    this.PTOLeft.setPosition(PTO_active[1]);
                    break;
                }
            }
        }
    }
}