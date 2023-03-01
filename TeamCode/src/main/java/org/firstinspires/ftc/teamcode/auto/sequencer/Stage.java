package org.firstinspires.ftc.teamcode.auto.sequencer;

import java.util.Optional;
import java.util.function.*;

public class Stage<T extends StageState> {
    String name;
    private final Consumer<T> action;
    private Consumer<T> startAction;
    private Consumer<T> finishAction;
    private Predicate<T> isEndPredicate;
    private Function<T, Optional<Stage<T>>> nextStageSupplier;
    boolean started = false;

    public Stage(String name, Consumer<T> action) {
        this.name = name.trim();
        this.action = action;
    }

    /**
     *
     * This method can be used to quickly and cleanly chain many Stages. If the next stage is conditional, use {@link #setNextStageFunction(Function) setNextStageFunction} to return the next stage dynamically.
     *
     * @param other The stage which will run after this one.
     * @return The inputted stage.
     */
    public Stage<T> nextStage(Stage<T> other) {
        if(other == null) {
            throw new IllegalStateException("The next stage cannot be null");
        }
        nextStageSupplier = state -> Optional.of(other);
        return other;
    }
    
    public Stage<T> setNextStageFunction(Function<T, Optional<Stage<T>>> supplier) {
        nextStageSupplier = supplier;
        return this;
    }
    
    public Stage<T> setIsEndPredicate(Predicate<T> predicate) {
        isEndPredicate = predicate;
        return this;
    }

    public Stage<T> setStartAction(Consumer<T> action) {
        startAction = action;
        return this;
    }

    public Stage<T> setFinishAction(Consumer<T> action) {
        finishAction = action;
        return this;
    }

    public boolean isEnd(T state) {
        if(isEndPredicate == null) {
            return false;
        }
        return isEndPredicate.test(state);
    }
    
    public void start(T state) {
        if(!started) {
            started = true;
            if(hasStartAction()) {
                startAction.accept(state);
            }
        }
    }
    
    public void finish(T state) {
        if(started) {
            started = false;
            if(hasFinishAction()) {
                finishAction.accept(state);
            }
        }
    }
    
    public void run(T state) {
        if(!started) {
            start(state);
        }
        action.accept(state);
    }
    
    public Optional<Stage<T>> calcNextStage(T state) {
        if(nextStageSupplier == null || isEndPredicate == null) {
            return Optional.empty();
        }
        return nextStageSupplier.apply(state);
    }
    
    public boolean hasNextStage() {
        return nextStageSupplier != null;
    }
    
    public boolean hasStartAction() {
        return startAction != null;
    }
    
    public boolean hasFinishAction() {
        return finishAction != null;
    }
    
    public String getName() {
        return name;
    }

    public void reset() {
        started = false;
    }

}