package org.firstinspires.ftc.teamcode.auto.sequencer;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.CustomVision;

public class RobotAutoState extends StageState {
    
    public CaidenRobot caiden;
    public CustomVision vision;
    public Telemetry telemetry;


    public static final PIDController forwardController = new PIDController(0.004, 0.001, 0.0005);
    public static final PIDController strafeController = new PIDController(0.01, 0.025, 0.0);
    public static final ProfiledPIDController rangeSensorController = new ProfiledPIDController(0.06, 0.002, 0.0, new TrapezoidProfile.Constraints(1, 0.3));
    public static final PIDController anglePID = new PIDController(0.027, 0.006, 0.002);

    public double power;
    public double pivot;
    public double heading;

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
    
    public  String recognizedSignal = "";
    
    //                          cone5,  cone4, etc
    public int[] coneHeight = {190,  180, 140, 70,  35};
    public int currentCone = 0;
    public int maxCone = 2;
    
    public void reset() {

        vision = null;
        caiden = null;
        currentCone = 0;
        robotInPosition = false;
        shouldEnd = false;
        seenConeLine = false;
        recognizedSignal = "";
    }
    
}