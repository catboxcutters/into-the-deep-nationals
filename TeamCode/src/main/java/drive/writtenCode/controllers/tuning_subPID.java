package drive.writtenCode.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import drive.writtenCode.RobotMap;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Testers")
public class tuning_subPID extends LinearOpMode {

    public static double maxSpeed = 0.7;
    public static double Kp = 0;//0.00325
    public static double Ki = 0;//0.0022
    public static double Kd = 0; //0.001
    public static double RetractedPosition = 0 , ExtendedPosition = -19000;
    public static double vMax = 0, AccMax = 0, JerkMax =0 , EndPos = 700;
    int TargetLift = 0;
    ElapsedTime timerPID = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {

        ElapsedTime changePositions = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap robot = new RobotMap(hardwareMap);

        SimplePIDController hello = new SimplePIDController(Kp,Ki,Kd);
        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime now = new ElapsedTime();
        now.reset();
        telemetry.update();
        hello.targetValue = RetractedPosition;
        while (!isStopRequested() && opModeIsActive())
        {
            int ColectarePosition = robot.encoderSlides.getCurrentPosition();
            double powerColectare = hello.update(ColectarePosition);
            powerColectare = Math.max(-maxSpeed,Math.min(powerColectare,maxSpeed));
            robot.slidesLeft.setPower(-powerColectare);
            robot.slidesMid.setPower(-powerColectare);
            robot.slidesRight.setPower(-powerColectare);
            if (changePositions.seconds()>4)
            {
                if (hello.targetValue == RetractedPosition )
                {
                    hello.targetValue = ExtendedPosition;
                }
                else
                {
                    hello.targetValue = RetractedPosition;
                }
                changePositions.reset();
            }
            telemetry.addData("ColectareEncoder", ColectarePosition);
            telemetry.addData("powerColectare", powerColectare);
            telemetry.addData("TargetLift",hello.targetValue);
            telemetry.addData("Error", hello.measuredError(ColectarePosition));
            telemetry.addData("power curent", robot.slidesLeft.getPower());
            telemetry.addData("Kp ", Kp);
            telemetry.addData("Kd ", Kd);
            telemetry.addData("Ki ", Ki);
            telemetry.addData("MaxSpeed ", hello.maxOutput);
            if (Kp!=hello.p || Kd!=hello.d || Ki!=hello.i || maxSpeed !=hello.maxOutput)
            {
                hello.p = Kp;
                hello.d = Kd;
                hello.i = Ki;
                hello.maxOutput = maxSpeed;
            }
            telemetry.update();
        }
    }
}