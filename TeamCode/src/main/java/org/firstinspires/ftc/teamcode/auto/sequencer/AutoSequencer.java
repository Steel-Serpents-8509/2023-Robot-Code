package org.firstinspires.ftc.teamcode.auto.sequencer;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;


public class AutoSequencer<T extends StageState> {
    
    private boolean started = false;
    private boolean stageDebugging = false;
    private BooleanSupplier moveToNextStage = null;
    T sharedState;

    private Stage<T> doNothingStage = new Stage<>("Do nothing", ignored -> {});
    Stage<T> currentStage = doNothingStage;

    public AutoSequencer(T state) {
        sharedState = state;
    }

    public void setDoNothingStage(Stage<T> stage) {
        doNothingStage = stage;
    }

    public String getCurrentStageName() {
        return currentStage.getName();
    }
    
    public void reset() {
        started = false;
        currentStage = doNothingStage;
        disableStageDebugging();
    }
    
    public void start(Stage<T> startingStage) {
        if(startingStage == null) {
            throw new IllegalStateException("Starting stage cannot be null");
        }
        currentStage = startingStage;
        started = true;
        if(sharedState.stageElapsedTime == null) {
            sharedState.stageElapsedTime = new ElapsedTime();
        }
        sharedState.stageElapsedTime.reset();
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
        currentStage = stage;
        currentStage.reset();
        sharedState.stageElapsedTime.reset();
    }

    private void finishCurrentStage() {
        currentStage.finish(sharedState);
    }

    private boolean isInInvalidRunningState() {
        return !started;
    }

    public boolean isStageDebugging() {
        return stageDebugging;
    }

    public void enableStageDebugging(BooleanSupplier moveToNextStageCondition) {
        stageDebugging = true;
        moveToNextStage = moveToNextStageCondition;
    }

    public void disableStageDebugging() {
        stageDebugging = false;
        moveToNextStage = null;
    }

}