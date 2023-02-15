package org.firstinspires.ftc.teamcode.auto_sequencer;

import java.util.function.*;

public class Stage<T extends StageState> {
    int id;
    private final Consumer<T> action;
    private Consumer<T> startAction;
    private Consumer<T> finishAction;
    private Predicate<T> isEndPredicate;
    private Function<T, Integer> nextStageSupplier;
    boolean started = false;
    
    public Stage(int id, Consumer<T> action, Consumer<T> startAction, Consumer<T> finishAction, Predicate<T> isEndPredicate, Function<T, Integer> nextStageSupplier) {
        this.id = id;
        this.action = action;
        this.isEndPredicate = isEndPredicate;
        this.nextStageSupplier = nextStageSupplier;
        this.startAction = startAction;
        this.finishAction = finishAction;
    }
    
    public Stage(int id, Consumer<T> action, Consumer<T> startAction, Consumer<T> finishAction, Predicate<T> isEndPredicate) {
        this(id, action, startAction, finishAction, isEndPredicate, null);
    }
    
    public Stage(Consumer<T> action, Consumer<T> startAction, Consumer<T> finishAction, Predicate<T> isEndPredicate) {
        this(0, action, startAction, finishAction, isEndPredicate, null);
    }
    
    public Stage(int id, Consumer<T> action, Consumer<T> startAction, Consumer<T> finishAction) {
        this(id, action, startAction, finishAction, null, null);
    }
    
    public Stage(int id, Consumer<T> action, Predicate<T> isEndPredicate, Function<T, Integer> nextStageSupplier) {
        this(id, action, null, null, isEndPredicate, nextStageSupplier);
    }
    
    public Stage(int id, Consumer<T> action, Consumer<T> startAction, Predicate<T> isEndPredicate) {
        this(id, action, startAction, null, isEndPredicate, null);
    }
    
    public Stage(Consumer<T> action, Consumer<T> startAction, Predicate<T> isEndPredicate) {
        this(0, action, startAction, null, isEndPredicate, null);
    }
    
    public Stage(int id, Consumer<T> action, Predicate<T> isEndPredicate) {
        this(id, action, null, null, isEndPredicate, null);
    }
    
    public Stage(Consumer<T> action, Predicate<T> isEndPredicate) {
        this(0, action, null, null, isEndPredicate, null);
    }
    
    public Stage(int id, Consumer<T> action) {
        this(id, action, null, null, null, null);
    }
    
    public Stage(Consumer<T> action) {
        this(0, action, null, null, null, null);
    }
    
    public Stage<T> nextStage(Stage<T> other) {
        nextStageSupplier = state -> other.id;
        return other;
    }
    
    public Stage<T> setNextStageFunction(Function<T, Integer> supplier) {
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
    
    public int calcNextStage(T state) {
        if(nextStageSupplier == null || isEndPredicate == null) {
            return -100;
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
    
    public int getId() {
        return id;
    }
}