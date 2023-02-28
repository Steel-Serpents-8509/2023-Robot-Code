package org.firstinspires.ftc.teamcode.competition;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ButtonDebounce;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.AutoStages;
import org.firstinspires.ftc.teamcode.auto.sequencer.RobotAutoState;
import org.firstinspires.ftc.teamcode.auto.sequencer.Stage;
import java.util.Optional;

@Autonomous
public class ConePlaceAuto extends LinearOpMode {

    CaidenRobot caiden;
    
    ElapsedTime loopTime = new ElapsedTime();
    
    @SuppressLint("SdCardPath")
    @Override
    public void runOpMode() {
        
        AutoStages.sequencer.reset();
        AutoStages.state.reset();
        
        caiden = new CaidenRobot(hardwareMap);

        ButtonDebounce debounce = new ButtonDebounce();
        //AutoStages.sequencer.enableStageDebugging(() -> debounce.getButton(gamepad1.a));

        AutoStages.state.caiden = caiden;
        AutoStages.state.vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_shape_model.tflite");
        AutoStages.state.telemetry = telemetry;
        AutoStages.sequencer.setDoNothingStage(new Stage<>("caiden.stop()", state -> caiden.stop()));


        AutoStages.closeClawOnPreloadCone
        .nextStage(AutoStages.recognizeSignalWithTimeout)
        .nextStage(AutoStages.goRightToWall)
        .nextStage(AutoStages.goForwardToConeStackWithStartingCone)
        .nextStage(AutoStages.findConeLinePosition)
        .nextStage(AutoStages.strafeToBigPole)
        .nextStage(AutoStages.raiseConeToPole)
        .nextStage(AutoStages.lowerStartingConeOntoPole)
        .nextStage(AutoStages.openClawWithStartingCone)
        .nextStage(AutoStages.goBackToConeStack)


        .nextStage(AutoStages.grabCone)
        .nextStage(AutoStages.strafeToPoleFromStack)
        .nextStage(AutoStages.raiseConeToPole)
        .nextStage(AutoStages.lowerStartingConeOntoPole)
        .nextStage(AutoStages.openClawWithStartingCone)
        .nextStage(AutoStages.goBackToConeStack);


        AutoStages.goToZone1.nextStage(AutoStages.backupIntoZoneSlightly);
        AutoStages.goToZone2.nextStage(AutoStages.backupIntoZoneSlightly);
        AutoStages.goToZone3.nextStage(AutoStages.backupIntoZoneSlightly);

        AutoStages.grabCone.setNextStageFunction(state -> {
            if(state.currentCone >= state.maxCone) {
                return Optional.of(AutoStages.goToZone);
            } else {
                return Optional.of(AutoStages.strafeToPoleFromStack);
            }
        });

        AutoStages.goToZone.setNextStageFunction(state -> {
            Stage<RobotAutoState> ret;
            switch (state.recognizedSignal) {
                case "circle":
                    ret = AutoStages.goToZone1;
                    break;
                case "triangle":
                    ret = AutoStages.goToZone3;
                    break;
                case "square":
                case "":
                default:
                    ret = AutoStages.goToZone2;
            }
            return Optional.of(ret);
        });
        
        RobotAutoState.forwardController.setTolerance(30);
        RobotAutoState.strafeController.setTolerance(40);
        RobotAutoState.anglePID.setTolerance(2);
        RobotAutoState.rangeSensorController.setTolerance(2.2);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Auto Stage", AutoStages.sequencer.getCurrentStageName());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        caiden.setHeadlightPower(0.06);
        caiden.changeP(0.002);
        caiden.changeI(0.00045);
        waitForStart();
        caiden.closeClaw();

        AutoStages.sequencer.start(AutoStages.closeClawOnPreloadCone);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            AutoStages.sequencer.run();
            updateTelemetry();
        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("Current Auto Stage Name", AutoStages.sequencer.getCurrentStageName());
        telemetry.addData("Auto Stage Time", AutoStages.sequencer.getTimeInStageMS());
        telemetry.addData("Power", AutoStages.state.power);
        telemetry.addData("Pivot", AutoStages.state.pivot);
        telemetry.addData("Recognized Signal", AutoStages.state.recognizedSignal);
  //      telemetry.addData("Seeing Line", AutoStages.seeingConeLine());
    //    telemetry.addData("Seen Line", AutoStages.state.seenConeLine);
        telemetry.addData("Current Cone", AutoStages.state.currentCone);
        telemetry.addData("Max Cone", AutoStages.state.maxCone);
  //      telemetry.addData("elevator height", caiden.getElevatorHeight());
        caiden.updateTelemetry(telemetry);
        telemetry.addData("Loop Time", loopTime.milliseconds());
        loopTime.reset();
        telemetry.update();
    }
    
}
    


