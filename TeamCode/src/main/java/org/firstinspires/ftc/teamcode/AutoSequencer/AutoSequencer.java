package org.firstinspires.ftc.teamcode.AutoSequencer;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AutoSequencer.*;
import java.util.function.*;
import java.util.*;


public class AutoSequencer<T extends StageState> {
    
    private int currentStageId = 0;
    private boolean started = false;
    T sharedState;
    Map<Integer, Stage<T>> stages = new HashMap<>();
    
    private static final int DO_NOTHING_STAGE_ID = -100;
    private Stage<T> doNothingStage;
    
    public AutoSequencer(T state) {
        sharedState = state;
        
        doNothingStage = new Stage<>(DO_NOTHING_STAGE_ID, (time) -> {});
    } 

    public void setDoNothingStage(Stage<T> stage) {
        stage.id = DO_NOTHING_STAGE_ID;
        doNothingStage = stage;
    }

    public void addStage(int id, Consumer<T> action, Predicate<T> isEnd, Function<T, Integer> calcNextStage) {
        addStage(new Stage<T>(id, action, isEnd, calcNextStage));
    }
    
    public void addStage(Stage<T> stage) {
        if(stages.containsKey(stage.getId())) {
            stage.id = stages.keySet().stream().max(Integer::compare).get() + 1;
        }
        stages.put(stage.getId(), stage);
    }
    
    public int getcurrentStageId() {
        return currentStageId;
    }
    
    public void reset() {
        stages.clear();
    }
    
    public void start() {
        start(0);
    } 
    
    public void start(int startingStage) {
        currentStageId = startingStage;
        addStage(doNothingStage);
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
        /*if(isError()) {
            updateCurrentStage(DO_NOTHING_STAGE_ID);
        }*/
        
        Stage<T> currentStage = stages.get(currentStageId);
        
        // If we are done,
        if(currentStage.isEnd(sharedState)) {
            if(currentStage.hasNextStage()) {
                // This runs finishStage() and updates the currentStageId
                updateCurrentStage(currentStage.calcNextStage(sharedState));
            } else {
                // This runs finishStage() and updates the currentStageId
                updateCurrentStage(DO_NOTHING_STAGE_ID);
            }
        } else {
            currentStage.run(sharedState);
        }
    }
    
    private void updateCurrentStage(int id) {
        finishStage();
        currentStageId = id;
        sharedState.stageElapsedTime.reset();
    }
    
    private void finishStage() {
        stages.get(currentStageId).finish(sharedState);
    }
    
    public boolean isError() {
        if(!started) {
            return true;
        } else if(!stages.containsKey(currentStageId)) {
            return true;
        }

        return false;
    }
    
}