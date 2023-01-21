package org.firstinspires.ftc.teamcode.AutoSequencer;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.CustomVision;

public class RobotAutoState extends StageState {
    
    public CaidenRobot caiden;
    public CustomVision vision;


    public static final PIDController forwardController = new PIDController(0.004, 0.001, 0.0004);
    public static final PIDController strafeController = new PIDController(0.01, 0.025, 0.0);
    public static final PIDController rangeSensorController = new PIDController(0.06, 0.02, 0.0003);
    //public static final PIDController anglePID = new PIDController(0.045, 0.02, 0.002);
    public static final PIDController anglePID = new PIDController(0.03, 0.006, 0.002);

    public double power;
    public double pivot;
    public double heading;
    
    public ElapsedTime elapsedStageTime;
    public ElapsedTime elapsedTimeInPosition = new ElapsedTime();
    public boolean robotInPosition = false;
    public boolean shouldEnd = false;
    
    public boolean seenConeLine = false;
    
    // 3600 = 72.5in
    // 1800 = 36.25in
    // 1800 + 1800 = 70in
    /*
        1800 = 35.5 
        1800 + 1800 = 71
    */
    public int distance = 3600;
    public int distanceToWall = 18;
    
    public  static double DIST_TO_WALL = 14.8;
    public  static int ELEVATOR_TOP = 830;
    public  String recognizedSignal = ""; 
    
    //                          cone5,  cone4, etc
    public int coneHeight[] =       {172,  140, 105, 70,  35};
    //double robotDistance[] =    {25, 23,    21,  14.2,  12.4}; 
    public int currentCone = -1;
    public int maxCone = 2; 
    
    public void reset() {
        
        currentCone = 0;
        robotInPosition = false;
        shouldEnd = false;
        seenConeLine = false;
    }
    
}