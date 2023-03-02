package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.auto.sequencer.RobotAutoState;

import java.util.Objects;
import java.util.function.Predicate;

public class EndPredicates {
    private EndPredicates() { throw new IllegalStateException("Utility Class"); }
    /*
     *
     *  Predicates:
     *  Conditions to leaving the stage
     *
     */
    public static final Predicate<RobotAutoState> recognizeSignalIsEndPredicate = autoState -> !Objects.equals(autoState.recognizedSignal, "") || autoState.stageElapsedTime.milliseconds() > 2500;

    public static final Predicate<RobotAutoState> forwardControllerIsEndPredicate = autoState -> {
        if(RobotAutoState.forwardController.atSetPoint() && RobotAutoState.anglePID.atSetPoint() && !autoState.robotInPosition) {
            autoState.elapsedTimeInPosition.reset();
            autoState.robotInPosition = true;
        } else if(!RobotAutoState.forwardController.atSetPoint() && autoState.robotInPosition) {
            autoState.robotInPosition = false;
        } else return RobotAutoState.forwardController.atSetPoint() && autoState.robotInPosition && autoState.elapsedTimeInPosition.milliseconds() > 400;
        return false;
    };
    public static final Predicate<RobotAutoState> strafeControllerIsEndPredicate = autoState -> {
        if(RobotAutoState.strafeController.atSetPoint() && !autoState.robotInPosition) {
            autoState.stageElapsedTime.reset();
            autoState.robotInPosition = true;
        } else if(!RobotAutoState.strafeController.atSetPoint() && autoState.robotInPosition) {
            autoState.robotInPosition = false;
        } else return RobotAutoState.strafeController.atSetPoint() && autoState.robotInPosition && autoState.stageElapsedTime.milliseconds() > 400;
        return false;
    };

    public static final Predicate<RobotAutoState> profiledStrafeControllerIsEndPredicate = autoState -> {
        if(RobotAutoState.profiledStrafeController.atGoal() && !autoState.robotInPosition) {
            autoState.stageElapsedTime.reset();
            autoState.robotInPosition = true;
        } else if(!RobotAutoState.profiledStrafeController.atGoal() && autoState.robotInPosition) {
            autoState.robotInPosition = false;
        } else return RobotAutoState.profiledStrafeController.atGoal() && autoState.robotInPosition && autoState.stageElapsedTime.milliseconds() > 400;
        return false;
    };
    public static final Predicate<RobotAutoState> rangeControllerIsEndPredicate = autoState -> {
        if(RobotAutoState.rangeSensorController.atGoal() && RobotAutoState.anglePID.atSetPoint() && !autoState.robotInPosition) {
            autoState.stageElapsedTime.reset();
            autoState.robotInPosition = true;
        } else if(!RobotAutoState.rangeSensorController.atGoal() && autoState.robotInPosition) {
            autoState.robotInPosition = false;
        } else return RobotAutoState.rangeSensorController.atGoal() && autoState.robotInPosition && autoState.stageElapsedTime.milliseconds() > 400;
        return false;
    };

    public static final Predicate<RobotAutoState> robotStateIsEndPredicate = autoState -> {
        if(autoState.shouldEnd) {
            autoState.shouldEnd = false;
            return true;
        }
        return false;
    };
    public static final Predicate<RobotAutoState> elevatorIsEndPredicate = autoState -> {
        if(autoState.caiden.elevatorIsInPosition()) {
            return true;
        }
        return false;
    };
}
