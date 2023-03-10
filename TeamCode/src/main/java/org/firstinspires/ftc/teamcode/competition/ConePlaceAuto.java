package org.firstinspires.ftc.teamcode.competition;

import android.view.ViewDebug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.AutoSequencer.*;
import java.util.function.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class ConePlaceAuto extends LinearOpMode {

    CaidenRobot caiden;
    
    private CustomVision vision;
    
    /*PIDController forwardController = new PIDController(0.0016, 0.00, 0.0);
    PIDController strafeController = new PIDController(0.01, 0.02, 0.0);
    PIDController rangeSensorController = new PIDController(0.04, 0.017, 0.0);
    PIDController anglePID = new PIDController(0.012, 0.02, 0.001);*/
    
    
    // 3600 = 72.5in
    // 1800 = 36.25in
    // 1800 + 1800 = 70in
    /*
        1800 = 35.5 
        1800 + 1800 = 71
    */
    Integer distance = 3600;
    
    private static double DIST_TO_WALL = 14.8;
    private static int ELEVATOR_TOP = 830;
    
    //                          cone5,  cone4, etc
    int coneHeight[] =       {160,  140, 105, 70,  35};
    //double robotDistance[] =    {25, 23,    21,  14.2,  12.4};
    ElapsedTime loopTime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        
        AutoStages.sequencer.reset();
        AutoStages.state.reset();
        
        caiden = new CaidenRobot(hardwareMap);
        AutoStages.state.caiden = caiden;
        vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_shape_model.tflite");
        AutoStages.state.vision = vision;
        //AutoStages.state.vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_original_sleeve_model.tflite");
        AutoStages.state.telemetry = telemetry;
        
        AutoStages.sequencer.setDoNothingStage(new Stage<>((state) -> caiden.stop()));
        
        
        AutoStages.closeClawOnPreloadCone
        .nextStage(AutoStages.recognizeSignalWithTimeout)
        .nextStage(AutoStages.goRightToWall)
        .nextStage(AutoStages.goForwardToConeStackWithStartingCone)
        .nextStage(AutoStages.findConeLinePosition)
        .nextStage(AutoStages.backupToShortPoleWithStartingCone)
        .nextStage(AutoStages.lowerStartingConeOntoPole)
        .nextStage(AutoStages.openClawWithStartingCone);
        
        AutoStages.openClawWithStartingCone
        .nextStage(AutoStages.goBackToConeStack);
        
        AutoStages.lineUpWithConeStack
        .nextStage(AutoStages.approachConeStack)
        .nextStage(AutoStages.grabCone)
        .nextStage(AutoStages.liftElevatorToClearConeStack)
        .nextStage(AutoStages.backupToShortPole)
        .nextStage(AutoStages.lowerConeOntoPole)
        .nextStage(AutoStages.openClaw)
        .nextStage(AutoStages.clearLowGoal)
        .nextStage(AutoStages.goBackToConeStack)
        .nextStage(AutoStages.approachConeStack);

        AutoStages.goToZone1.nextStage(AutoStages.backupIntoZoneSlightly);
        AutoStages.goToZone2.nextStage(AutoStages.backupIntoZoneSlightly);
        AutoStages.goToZone3.nextStage(AutoStages.backupIntoZoneSlightly);

        AutoStages.liftElevatorToClearConeStack.setNextStageFunction((state) -> {
            if(state.currentCone >= state.maxCone) {
                return AutoStages.goToZone.getId();
            } else {
                return AutoStages.backupToShortPole.getId();
            }
        });

        /*
        AutoStages.goBackToConeStack.setNextStageFunction((state) -> {
            if(state.currentCone >= state.maxCone) {
                return AutoStages.goToZone.getId();
            } else {
                return AutoStages.approachConeStack.getId();
            }
        });*/

        AutoStages.goToZone.setNextStageFunction((state) -> {
            int ret;
            switch (state.recognizedSignal) {
                case "circle":
                    ret = AutoStages.goToZone1.getId();
                    break;
                case "triangle":
                    ret = AutoStages.goToZone3.getId();
                    break;
                case "square":
                case "":
                default:
                    ret = AutoStages.goToZone2.getId();
            }
            return ret;
        });
        
        AutoStages.sequencer.addStage(AutoStages.closeClawOnPreloadCone);
        AutoStages.sequencer.addStage(AutoStages.recognizeSignalWithTimeout);
        AutoStages.sequencer.addStage(AutoStages.goRightToWall);
        AutoStages.sequencer.addStage(AutoStages.goForwardToConeStackWithStartingCone);
        AutoStages.sequencer.addStage(AutoStages.findConeLinePosition);
        AutoStages.sequencer.addStage(AutoStages.backupToShortPoleWithStartingCone);
        AutoStages.sequencer.addStage(AutoStages.lowerStartingConeOntoPole);
        AutoStages.sequencer.addStage(AutoStages.openClawWithStartingCone);
        
        AutoStages.sequencer.addStage(AutoStages.lineUpWithConeStack);
        AutoStages.sequencer.addStage(AutoStages.approachConeStack);
        AutoStages.sequencer.addStage(AutoStages.grabCone);
        AutoStages.sequencer.addStage(AutoStages.liftElevatorToClearConeStack);
        AutoStages.sequencer.addStage(AutoStages.backupToShortPole);
        AutoStages.sequencer.addStage(AutoStages.lowerConeOntoPole);
        AutoStages.sequencer.addStage(AutoStages.openClaw);
        AutoStages.sequencer.addStage(AutoStages.clearLowGoal);
        AutoStages.sequencer.addStage(AutoStages.goBackToConeStack);
        AutoStages.sequencer.addStage(AutoStages.goToZone);
        AutoStages.sequencer.addStage(AutoStages.goToZone1);
        AutoStages.sequencer.addStage(AutoStages.goToZone2);
        AutoStages.sequencer.addStage(AutoStages.goToZone3);
        AutoStages.sequencer.addStage(AutoStages.backupIntoZoneSlightly);
        

        //vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/original_sleeve_good_data.tflite");
        //vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_shape_model.tflite");
        RobotAutoState.forwardController.setTolerance(30);
        RobotAutoState.strafeController.setTolerance(40);
        RobotAutoState.anglePID.setTolerance(2);
        RobotAutoState.rangeSensorController.setTolerance(2.2);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Stage ID", AutoStages.closeClawOnPreloadCone.getId());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        caiden.enableHeadlight();
        caiden.setHeadlightPower(0.06);
        waitForStart();
        caiden.closeClaw();


        //recognizeSignalZone();
        //caiden.disableHeadlight();
        //caiden.setHeadlightPower(0);
        
        AutoStages.sequencer.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            
            AutoStages.sequencer.run();
            updateTelemetry();
            

        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("Auto Stage", AutoStages.sequencer.getcurrentStageId());
        telemetry.addData("Auto Stage Time", AutoStages.sequencer.getTimeInStageMS());
        telemetry.addData("Power", AutoStages.state.power);
        telemetry.addData("Pivot", AutoStages.state.pivot);
        telemetry.addData("Recognized Signal", AutoStages.state.recognizedSignal);
        telemetry.addData("Seeing Line", AutoStages.seeingConeLine());
        telemetry.addData("Seen Line", AutoStages.state.seenConeLine);
        telemetry.addData("Current Cone", AutoStages.state.currentCone);
        telemetry.addData("Max Cone", AutoStages.state.maxCone);
        caiden.updateTelemetry(telemetry);
        telemetry.addData("Loop Time", loopTime.milliseconds());
        loopTime.reset();
        telemetry.update();
    }
    
    public void postTelemetry() {
        
        caiden.updateTelemetry(telemetry);
        //telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        if(vision != null) {
            vision.updateTelemetry(telemetry);
        }
        telemetry.update();
    }
    
    
    
}
    


