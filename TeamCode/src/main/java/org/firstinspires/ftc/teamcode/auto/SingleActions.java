package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.auto.sequencer.RobotAutoState;

import java.util.function.Consumer;

public class SingleActions {
    private SingleActions() { throw new IllegalStateException("Utility Class"); }

    // One off helper actions
    public static final Consumer<RobotAutoState> resetDrivetrain = autoState -> autoState.caiden.resetDrivetrain();
}
