package drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import drive.writtenCode.RobotMap;

public class ClimbController {
    public enum ClimbStatus{
        STOP,
        UP,
        DOWN,
    }
    public ClimbStatus currentStatus = ClimbController.ClimbStatus.STOP;
    public ClimbController.ClimbStatus previousStatus = null;

    private CRServo  hangingRight= null;
    private CRServo  hangingLeft= null;

    public ClimbController(RobotMap robot)
    {
       hangingLeft = robot.climbLeft;
        hangingRight = robot.climbRight;
    }
    public void update()
    {
        if (currentStatus != previousStatus)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case STOP:
                {
                    hangingRight.setPower(0);
                    hangingLeft.setPower(0);
                    break;
                }
                case UP:
                {
                    hangingRight.setPower(1);
                    hangingLeft.setPower(-1);
                    break;
                }
                case DOWN:
                {
                    hangingRight.setPower(-1);
                    hangingLeft.setPower(1);
                    break;
                }
            }
        }
    }}