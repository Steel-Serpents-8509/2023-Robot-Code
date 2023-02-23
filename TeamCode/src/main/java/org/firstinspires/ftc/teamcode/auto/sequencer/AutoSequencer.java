package org.firstinspires.ftc.teamcode.auto.sequencer;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;
import java.util.function.BooleanSupplier;


public class AutoSequencer<T extends StageState> {
    
    private boolean started = false;
    private boolean stageDebugging = false;
    private BooleanSupplier moveToNextStage = null;
    T sharedState;

    Optional<Stage<T>> currentStageOpt = Optional.empty();
    private Stage<T> doNothingStage;
    
    public AutoSequencer(T state) {
        sharedState = state;
        
        doNothingStage = new Stage<>("Do nothing", ignored -> {});
    } 

    public void setDoNothingStage(Stage<T> stage) {
        doNothingStage = stage;
    }

    public String getCurrentStageName() {
        return currentStageOpt.map(Stage::getName).orElse("No current stage");
    }
    
    public void reset() {
        started = false;
        currentStageOpt = Optional.empty();
    }
    
    public void start() {
        started = true;
        if(sharedState.stageElapsedTime == null) {
            sharedState.stageElapsedTime = new ElapsedTime();
        }
        sharedState.stageElapsedTime.reset();
    }
    
    public void start(Stage<T> startingStage) {
        currentStageOpt = Optional.of(startingStage);
        start();
    }

    public boolean isStart() {
        return started;
    }
    
    public double getTimeInStageMS() {
        return sharedState.stageElapsedTime.milliseconds();
    }
    
    public void run() {
        // isError checks that currentStage exists
        if(isInInvalidRunningState()) {
            updateCurrentStage(doNothingStage);
        }

        Stage<T> currentStage = getCurrentStage();
        
        // If the current stage is ending, call updateCurrentStage and update currentStage
        if(currentStage.isEnd(sharedState) && (!stageDebugging || moveToNextStage.getAsBoolean())) {
            updateCurrentStage(currentStage.calcNextStage(sharedState).orElse(doNothingStage));
        }

        currentStage.run(sharedState);
    }
    
    private void updateCurrentStage(Stage<T> stage) {
        if(started) {
            finishCurrentStage();
        }
        currentStageOpt = Optional.of(stage);
        sharedState.stageElapsedTime.reset();
    }

    private void finishCurrentStage() {
        getCurrentStage().finish(sharedState);
    }

    private Stage<T> getCurrentStage() {
        if(!currentStageOpt.isPresent()) {
            throw new IllegalStateException("Current stage doesn't exist");
        } else {
            return currentStageOpt.get();
        }
    }
    private boolean isInInvalidRunningState() {
        return !started || !currentStageOpt.isPresent();
    }

    public boolean isStageDebugging() {
        return stageDebugging;
    }

    public void enableStageDebugging(BooleanSupplier supplier) {
        stageDebugging = true;
        moveToNextStage = supplier;
    }

    public void disableStageDebugging() {
        stageDebugging = false;
        moveToNextStage = null;
    }

}