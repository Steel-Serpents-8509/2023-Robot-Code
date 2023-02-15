package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.AutoStages;

import org.firstinspires.ftc.teamcode.auto_sequencer.*;

@Autonomous
public class TestingAuto extends LinearOpMode {

    CaidenRobot caiden;
    
    // 3600 = 72.5in
    // 1800 = 36.25in
    // 1800 + 1800 = 70in
    /*
        1800 = 35.5 
        1800 + 1800 = 71
    */


    @SuppressLint("SdCardPath")
    @Override
    public void runOpMode() {
        
        AutoStages.sequencer.reset();
        AutoStages.state.reset();
        
        caiden = new CaidenRobot(hardwareMap);
        AutoStages.state.caiden = caiden;
        AutoStages.state.vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/black_shapes_good_videos.tflite");
        
        AutoStages.sequencer.setDoNothingStage(new Stage<>(state -> caiden.stop()));
        
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
        .nextStage(AutoStages.goBackToConeStack);
        
        
        AutoStages.goBackToConeStack.setNextStageFunction(state -> {
            if(state.currentCone >= state.maxCone) {
                return -100;
            } else {
                return AutoStages.approachConeStack.getId();
            }
        });
        
        
        AutoStages.sequencer.addStage(AutoStages.testElevator);
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
        
        
        RobotAutoState.forwardController.setTolerance(30);
        RobotAutoState.strafeController.setTolerance(30);
        RobotAutoState.anglePID.setTolerance(2);
        RobotAutoState.rangeSensorController.setTolerance(2.2);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Stage ID", AutoStages.closeClawOnPreloadCone.getId());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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
        caiden.updateTelemetry(telemetry);
        telemetry.update();
    }
}
    


