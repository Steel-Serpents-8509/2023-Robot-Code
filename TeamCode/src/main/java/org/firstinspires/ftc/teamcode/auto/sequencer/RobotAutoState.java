package org.firstinspires.ftc.teamcode.auto.sequencer;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.RobotProperties;

public class RobotAutoState extends StageState {
    
    public CaidenRobot caiden;
    public CustomVision vision;
    public Telemetry telemetry;

    public static final PIDController forwardController = new PIDController(0.004, 0.001, 0.0005);
    public static final PIDController strafeController = new PIDController(0.01, 0.025, 0.0);

    public static ProfiledPIDController rangeSensorController;
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
    public double distanceToWall = 18;
    
    public  String recognizedSignal = "";
    
    //                          cone5,  cone4, etc
    public int[] coneHeight = {155, 115, 95, 53,  0};
    public int currentCone = -1;
    public int maxCone = 2;
    
    public void reset() {

        vision = null;
        caiden = null;
        currentCone = -1;
        robotInPosition = false;
        shouldEnd = false;
        seenConeLine = false;
        recognizedSignal = "";

        PIDCoefficients rangeSensorPIDValues = RobotProperties.getPIDCoefficients("rangeSensorController", new PIDCoefficients(0.1, 0.0, 0.0));
        RobotAutoState.rangeSensorController = new ProfiledPIDController(rangeSensorPIDValues.p, rangeSensorPIDValues.i, rangeSensorPIDValues.d, new TrapezoidProfile.Constraints(RobotProperties.getDoubleValue("rangeSensorProfileVelocity", 0.8), RobotProperties.getDoubleValue("rangeSensorProfileAcceleration", 0.5)));

    }
    
}