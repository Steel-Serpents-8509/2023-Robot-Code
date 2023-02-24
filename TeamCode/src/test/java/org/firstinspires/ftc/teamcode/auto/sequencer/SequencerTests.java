package org.firstinspires.ftc.teamcode.auto.sequencer;




import static org.junit.jupiter.api.Assertions.assertThrows;


import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import java.util.function.Predicate;

public class SequencerTests {

    static class TestState extends StageState {
        int i1 = 0;
    }

    static void println(String str) {
        System.out.println(str);
    }

    static Predicate<TestState> alwaysTrue = sharedState -> true;

    static Stage<TestState> stage1 = new Stage<TestState>("Stage 1", sharedState -> {println("Stage 1!");}).setIsEndPredicate(alwaysTrue);
    static Stage<TestState> stage2 = new Stage<TestState>("Stage 2", sharedState -> {println("Stage 2!");}).setIsEndPredicate(alwaysTrue);
    static Stage<TestState> stage3 = new Stage<TestState>("Stage 3", sharedState -> {println("Stage 3!");}).setIsEndPredicate(alwaysTrue);

    @Test
    public void basicTest() {
        TestState testState = new TestState();
        AutoSequencer<TestState> sequencer = new AutoSequencer<>(testState);

        stage1.nextStage(stage2)
                .nextStage(stage3);

        sequencer.start(stage1);
    }

    @Test
    public void nullTest() {
        TestState testState = new TestState();
        AutoSequencer<TestState> sequencer = new AutoSequencer<>(testState);

        assertThrows(IllegalStateException.class, () -> stage1.nextStage(null));

        sequencer.start(stage1);
    }
}
