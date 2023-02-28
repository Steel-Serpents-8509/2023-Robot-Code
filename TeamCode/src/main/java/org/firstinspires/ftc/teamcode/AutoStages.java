package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.auto.EndPredicates.*;
import static org.firstinspires.ftc.teamcode.auto.SingleActions.*;

import org.firstinspires.ftc.teamcode.auto.sequencer.AutoSequencer;
import org.firstinspires.ftc.teamcode.auto.sequencer.RobotAutoState;
import org.firstinspires.ftc.teamcode.auto.sequencer.Stage;

import java.util.Comparator;
import java.util.function.*;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class AutoStages {
    private AutoStages() {
        throw new IllegalStateException("Utility Class");
    }

    public static final RobotAutoState state = new RobotAutoState();
    public static final AutoSequencer<RobotAutoState> sequencer = new AutoSequencer<>(state);

    public static final Consumer<RobotAutoState> actionRightRangeSensor = autoState -> {
        double measuredDistance = autoState.caiden.getDistance();
        autoState.telemetry.addData("Target Distance", autoState.distanceToWall);
        autoState.telemetry.addData("Measured Distance", measuredDistance);
        autoState.telemetry.addData("Setpoint Position", RobotAutoState.rangeSensorController.getSetpoint().position);
        autoState.telemetry.addData("Goal Position", RobotAutoState.rangeSensorController.getGoal().position);

        autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(measuredDistance), -0.7, 0.7);
        autoState.telemetry.addData("At Goal", RobotAutoState.rangeSensorController.atGoal());
        autoState.telemetry.addData("Position Error", RobotAutoState.rangeSensorController.getPositionError());
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
    };

    // Cone auto
    public static final Stage<RobotAutoState> goRightToWall = new Stage<>("Go to right wall with distance sensor", actionRightRangeSensor)
            .setStartAction(autoState -> {
                autoState.caiden.resetDrivetrain();
                autoState.distanceToWall = RobotProperties.getDoubleValue("autoDistanceToWall", 17.0);
                RobotAutoState.rangeSensorController.reset(autoState.caiden.getDistance());
                RobotAutoState.rangeSensorController.setGoal(autoState.distanceToWall);
            })
            .setFinishAction(resetDrivetrain)
            .setIsEndPredicate(rangeControllerIsEndPredicate);

    public static final Consumer<RobotAutoState> actionGoForwardToPosition = autoState -> {

        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
    };

    public static final Consumer<RobotAutoState> actionGoSidewaysToPosition = autoState -> {
        autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
    };


    public static final Stage<RobotAutoState> goForwardToConeStack = new Stage<>("Go forward to cone stack", actionGoForwardToPosition)
            .setStartAction(autoState -> autoState.distance = 1700)
            .setIsEndPredicate(forwardControllerIsEndPredicate);

    public static final Stage<RobotAutoState> goForwardToConeStackWithStartingCone = new Stage<RobotAutoState>("Go forward to cone stack holding starting cone", autoState -> {

        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
    })
            .setStartAction(autoState -> autoState.distance = 1700)
            .setIsEndPredicate(forwardControllerIsEndPredicate);


    public static final Stage<RobotAutoState> lineUpWithConeStack = new Stage<>("Line up with cone stack", actionGoForwardToPosition)
            .setFinishAction(resetDrivetrain)
            .setIsEndPredicate(forwardControllerIsEndPredicate);

    private static final int DISTANCE_TO_CONE = 9;
    public static final Stage<RobotAutoState> approachConeStack = new Stage<RobotAutoState>("Approach cone stack", autoState -> {
        autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(autoState.caiden.getDistance(), autoState.distanceToWall), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        autoState.caiden.lazyR();
        autoState.caiden.openClaw();
    })
            .setStartAction(autoState -> {
                autoState.caiden.resetDrivetrain();
                autoState.distanceToWall = DISTANCE_TO_CONE;
            })
            .setIsEndPredicate(autoState -> (rangeControllerIsEndPredicate.test(autoState) && autoState.caiden.elevatorIsInPosition())
            );

    public static final Stage<RobotAutoState> grabCone = new Stage<RobotAutoState>("Grab cone off cone stack", autoState -> {
        autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(autoState.caiden.getDistance(), autoState.distanceToWall), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.closeClaw();
        if (autoState.stageElapsedTime.milliseconds() < 400) {
            autoState.caiden.closeClaw();
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        } else {
            autoState.caiden.goToLowElevatorPosition();
            autoState.caiden.closeClaw();

        }


    })
            .setStartAction(autoState -> {
                autoState.caiden.resetDrivetrain();

            })
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 5000);

    public static final Stage<RobotAutoState> liftElevatorToClearConeStack = new Stage<RobotAutoState>("Lift elevator to clear cone stack", autoState -> {
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        // 915 is low goal
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.lazyR();
        autoState.caiden.closeClaw();
    })
            .setStartAction(autoState -> autoState.caiden.resetDrivetrain())
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 800);

    public static final Consumer<RobotAutoState> actionBackupToShortPole = autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.35, 0.35);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.closeClaw();
    };
    public static final Consumer<RobotAutoState> actionStrafeToBigPoleWithStartingCone = autoState -> {
        autoState.power = Range.clip(RobotAutoState.strafeController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.5, 0.5);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
    };

    public static final Consumer<RobotAutoState> actionRaiseToBigPole = autoState -> {
        autoState.power = Range.clip(RobotAutoState.strafeController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.5, 0.5);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToHighElevatorPosition();
        autoState.caiden.horizontalSlideOut();
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
    };
    public static final Stage<RobotAutoState> strafeToBigPole = new Stage<>("Strafe to big pole with starting cone", actionStrafeToBigPoleWithStartingCone)
            .setStartAction(autoState -> {
                autoState.caiden.resetDrivetrain();
                autoState.caiden.lazyS();
                autoState.distance = 1680;
            }).setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 7000);

    public static final Stage<RobotAutoState> strafeToPoleFromStack = new Stage<>("Strafe to big Pole", actionStrafeToBigPoleWithStartingCone)
            .setStartAction(autoState -> {
                autoState.caiden.resetDrivetrain();
                autoState.caiden.lazyS();
                autoState.distance = 1650;
            }).setIsEndPredicate(strafeControllerIsEndPredicate);

    public static final Consumer<RobotAutoState> actionLowerConeOntoPole = autoState -> {
        if (autoState.stageElapsedTime.milliseconds() > 400) {
            autoState.caiden.horizontalSlideOut();
            autoState.caiden.goToElevatorPosition(1000);

        }
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
    };

    public static final Stage<RobotAutoState> lowerStartingConeOntoPole = new Stage<>("Lower starting cone onto pole", actionLowerConeOntoPole)
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 5000);

    public static final Stage<RobotAutoState> raiseConeToPole = new Stage<>("Lower starting cone onto pole", actionRaiseToBigPole)
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 5000);

    public static final Stage<RobotAutoState> lowerConeOntoPole = new Stage<>("Lower cone onto pole", actionLowerConeOntoPole)
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 600);

    public static final Consumer<RobotAutoState> actionOpenClaw = autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToElevatorPosition(900);
        autoState.caiden.lazyS();
        autoState.caiden.openClaw();
    };

    public static final Stage<RobotAutoState> openClawWithStartingCone = new Stage<>("Open claw with starting cone", actionOpenClaw)
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 1000);

    public static final Stage<RobotAutoState> openClaw = new Stage<>("Open claw to drop cone", actionOpenClaw)
            .setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 400);

    public static final Stage<RobotAutoState> clearLowGoal = new Stage<RobotAutoState>("Raise elevator to clear low goal", autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyR();
        autoState.caiden.openClaw();
    }).setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 700);

    public static final Stage<RobotAutoState> goBackToConeStack = new Stage<RobotAutoState>("Go back to cone stack", autoState -> {
        double measuredDistance = autoState.caiden.getDistance();
        autoState.telemetry.addData("Target Distance", autoState.distanceToWall);
        autoState.telemetry.addData("Measured Distance", measuredDistance);
        autoState.telemetry.addData("Setpoint Position", RobotAutoState.rangeSensorController.getSetpoint().position);
        autoState.telemetry.addData("Goal Position", RobotAutoState.rangeSensorController.getGoal().position);
        autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(measuredDistance), -0.7, 0.7);
        autoState.telemetry.addData("At Goal", RobotAutoState.rangeSensorController.atGoal());
        autoState.telemetry.addData("Position Error", RobotAutoState.rangeSensorController.getPositionError());
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.horizontalSlideIn();
        autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        autoState.caiden.lazyR();
        autoState.caiden.openClaw();
    })
            .setStartAction(autoState -> {
                autoState.currentCone++;
                autoState.distanceToWall = RobotProperties.getDoubleValue("autoDistanceToCones", 7.0);
                RobotAutoState.rangeSensorController.reset(autoState.caiden.getDistance());
                RobotAutoState.rangeSensorController.setGoal(autoState.distanceToWall);
            })
            .setIsEndPredicate(rangeControllerIsEndPredicate);


    public static boolean seeingConeLine() {
        return state.caiden.colorSensorRed() > 500 || state.caiden.colorSensorBlue() > 500;
    }

    public static final Stage<RobotAutoState> testElevator = new Stage<>("Test elevator", autoState -> autoState.caiden.goToElevatorPosition(915));


    public static final Stage<RobotAutoState> closeClawOnPreloadCone = new Stage<RobotAutoState>("Close claw on preloaded cone", autoState -> {
        autoState.caiden.closeClaw();
        autoState.caiden.changeP(0.01);
        if (autoState.stageElapsedTime.milliseconds() > 1000) {
            autoState.caiden.goToLowElevatorPosition();
        }
    }).setIsEndPredicate(autoState -> autoState.stageElapsedTime.milliseconds() > 1400);


    public static final Stage<RobotAutoState> findConeLinePosition = new Stage<RobotAutoState>("Fine cone line position", autoState -> {
        if (seeingConeLine() && !autoState.seenConeLine) {
            autoState.seenConeLine = true;
        } else if (autoState.seenConeLine && !seeingConeLine()) {
            autoState.distance = autoState.caiden.getFRMotorCount() + 80;
            autoState.shouldEnd = true;
        }

        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);

        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();

    }).setStartAction(elapsed -> state.power = 0.2).setIsEndPredicate(robotStateIsEndPredicate);


    // Should have 1-2 second timeout
    public static final Stage<RobotAutoState> recognizeSignalWithTimeout = new Stage<RobotAutoState>("Recognize signal with timeout", autoState -> {


        autoState.caiden.goToLowElevatorPosition();
        autoState.caiden.closeClaw();

        //TODO: Figure out if this delay is really needed
        if (autoState.stageElapsedTime.milliseconds() < 300) {
            return;
        }

        List<Recognition> recognizedObjects = autoState.vision.lookForObject();

        // step through the list of recognitions and display image position/size information for each one
        // Note: "Image number" refers to the randomized image orientation/number
        for (Recognition recognition : recognizedObjects) {
            double col = (recognition.getLeft() + recognition.getRight()) / 2;
            double row = (recognition.getTop() + recognition.getBottom()) / 2;
            double width = Math.abs(recognition.getRight() - recognition.getLeft());
            double height = Math.abs(recognition.getTop() - recognition.getBottom());

            autoState.telemetry.addData("", " ");
            autoState.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            autoState.telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
            autoState.telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
        }
        if (!recognizedObjects.isEmpty()) {
            autoState.recognizedSignal = recognizedObjects.stream().max(Comparator.comparingDouble(Recognition::getConfidence)).get().getLabel();
        }
    })
            .setStartAction(autoState -> autoState.caiden.setHeadlightPower(0.06))
            .setFinishAction(autoState -> autoState.caiden.setHeadlightPower(0.0))
            .setIsEndPredicate(recognizeSignalIsEndPredicate);


    public static final Stage<RobotAutoState> goToZone = new Stage<RobotAutoState>("Go to zone", autoState -> {
    })
            .setIsEndPredicate(ignored -> true);


    public static final Stage<RobotAutoState> goToZone1 = new Stage<RobotAutoState>("Go to zone 1", autoState -> {
        autoState.power = Range.clip(RobotAutoState.strafeController.calculate(autoState.caiden.getFRMotorCount(), 2400), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
        if (autoState.stageElapsedTime.milliseconds() > 2000) {
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        }
    }).setStartAction(autoState -> autoState.caiden.reset()).setFinishAction(autoState -> autoState.caiden.reset()).setIsEndPredicate(strafeControllerIsEndPredicate);

    public static final Stage<RobotAutoState> goToZone2 = new Stage<RobotAutoState>("Go to zone 2", autoState -> {
        autoState.power = Range.clip(RobotAutoState.strafeController.calculate(autoState.caiden.getFRMotorCount(), 1200), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
        if (autoState.stageElapsedTime.milliseconds() > 2000) {
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        }
    }).setStartAction(autoState -> autoState.caiden.reset()).setIsEndPredicate(strafeControllerIsEndPredicate);

    public static final Stage<RobotAutoState> goToZone3 = new Stage<RobotAutoState>("Go to zone 3", autoState -> {
        autoState.power = 0;
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
        if (autoState.stageElapsedTime.milliseconds() > 2000) {
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        }
    }).setStartAction(autoState -> autoState.caiden.reset()).setIsEndPredicate(strafeControllerIsEndPredicate);

    public static final Stage<RobotAutoState> backupIntoZoneSlightly = new Stage<RobotAutoState>("Backup into zone slightly", autoState -> {
        autoState.telemetry.addData("Backing up distance: ", autoState.distance);
        autoState.distance = -300;
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.goToElevatorPosition(0);
    }).setStartAction(autoState -> {
        autoState.caiden.resetDrivetrain();
        autoState.caiden.reset();
        autoState.distance = -300;
    });
}
