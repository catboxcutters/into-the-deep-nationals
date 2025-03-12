package drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import drive.writtenCode.RobotMap;
import org.openftc.easyopencv.*;
import drive.writtenCode.pipelines.edge_pipeline;

public class DistanceSensorsController {

    private AnalogInput distance_sensor1, distance_sensor2;

    public DistanceSensorsController(RobotMap robot) {
    this.distance_sensor1 = robot.distance_sensor1;
    this.distance_sensor2 = robot.distance_sensor2;
    }
    public double get_distance_1()
    {
        return(this.distance_sensor1.getVoltage()*520/3.3);
    }
    public double get_distance_2()
    {
        return(this.distance_sensor2.getVoltage()*520/3.3);
    }
    public double get_average_distance()
    {
        return((this.distance_sensor1.getVoltage()*520/3.3+this.distance_sensor2.getVoltage()*520/3.3)/2);
    }
}
