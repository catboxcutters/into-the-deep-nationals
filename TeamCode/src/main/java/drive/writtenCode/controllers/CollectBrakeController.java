package drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import drive.writtenCode.RobotMap;

@Config
public class CollectBrakeController {
    public enum BrakeStatus {
        OPEN,
        CLOSE,
    }
    public BrakeStatus currentStatus = BrakeStatus.OPEN;
    public BrakeStatus previousStatus=null;

    public static double open=0.5;
    public static double closed = 0;
    public Servo brake = null;


    public CollectBrakeController(RobotMap robot){
        this.brake = robot.collect_brake;
    }
    public void update(){
        if(currentStatus!=previousStatus)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case OPEN:
                {
                    this.brake.setPosition(open);
                    break;
                }
                case CLOSE:
                {
                    this.brake.setPosition(closed);
                    break;
                }
            }
        }
    }
}