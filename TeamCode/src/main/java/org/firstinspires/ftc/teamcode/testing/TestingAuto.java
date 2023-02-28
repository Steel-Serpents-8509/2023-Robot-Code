package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoStages;
import org.firstinspires.ftc.teamcode.ButtonDebounce;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.auto.sequencer.RobotAutoState;
import org.firstinspires.ftc.teamcode.auto.sequencer.Stage;

import java.util.Optional;

@Autonomous
public class TestingAuto extends LinearOpMode {

    CaidenRobot caiden;
    
    ElapsedTime loopTime = new ElapsedTime();
    
    @SuppressLint("SdCardPath")
    @Override
    public void runOpMode() {
        
        AutoStages.sequencer.reset();
        AutoStages.state.reset();
        
        caiden = new CaidenRobot(hardwareMap);

        //ButtonDebounce debounce = new ButtonDebounce();
        //AutoStages.sequencer.enableStageDebugging(() -> debounce.getButton(gamepad1.a));

        AutoStages.state.caiden = caiden;
        AutoStages.state.vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_shape_model.tflite");
        AutoStages.state.telemetry = telemetry;
        
        AutoStages.sequencer.setDoNothingStage(new Stage<>("caiden.stop()", state -> caiden.stop()));

        RobotAutoState.forwardController.setTolerance(30);
        RobotAutoState.strafeController.setTolerance(40);
        RobotAutoState.anglePID.setTolerance(2);
        RobotAutoState.rangeSensorController.setTolerance(1.6);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Auto Stage", AutoStages.sequencer.getCurrentStageName());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        caiden.setHeadlightPower(0.06);
        waitForStart();
        caiden.closeClaw();

        /*AutoStages.goRightToWall.nextStage(AutoStages.approachConeStack)
                .nextStage(AutoStages.backupToShortPole);*/



        AutoStages.sequencer.start(AutoStages.goBackToConeStack);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            AutoStages.sequencer.run();
            updateTelemetry();
        }
        caiden.stop();
    }
    
    private void updateTelemetry() {
        telemetry.addData("Current Auto Stage Name", AutoStages.sequencer.getCurrentStageName());
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
    
}
    


