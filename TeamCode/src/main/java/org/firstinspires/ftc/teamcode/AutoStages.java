package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.AutoSequencer.*;

import java.util.Comparator;
import java.util.Objects;
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

    
    /*
     *  
     *  Predicates:
     *  Conditions to leaving the stage
     * 
     */
    public static final Predicate<RobotAutoState> recognizeSignalIsEndPredicate = autoState -> !Objects.equals(autoState.recognizedSignal, "") || autoState.stageElapsedTime.milliseconds() > 2500;

    public static final Predicate<RobotAutoState> forwardControllerIsEndPredicate = autoState -> {
        if(RobotAutoState.forwardController.atSetPoint() && RobotAutoState.anglePID.atSetPoint() && !autoState.robotInPosition) {
            autoState.stageElapsedTime.reset();
            autoState.robotInPosition = true;
        } else if(!RobotAutoState.forwardController.atSetPoint() && autoState.robotInPosition) {
            autoState.robotInPosition = false;
        } else return RobotAutoState.forwardController.atSetPoint() && autoState.robotInPosition && autoState.stageElapsedTime.milliseconds() > 400;
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
    public static final Predicate<RobotAutoState> rangeControllerIsEndPredicate = autoState -> {
        if(RobotAutoState.rangeSensorController.atSetPoint() && RobotAutoState.anglePID.atSetPoint() && !autoState.robotInPosition) {
            autoState.stageElapsedTime.reset();
            autoState.robotInPosition = true;
        } else if(!RobotAutoState.rangeSensorController.atSetPoint() && autoState.robotInPosition) {
            autoState.robotInPosition = false;
        } else return RobotAutoState.rangeSensorController.atSetPoint() && autoState.robotInPosition && autoState.stageElapsedTime.milliseconds() > 400;
        return false;
    };

    public static final Predicate<RobotAutoState> robotStateIsEndPredicate = autoState -> {
        if(autoState.shouldEnd) {
            autoState.shouldEnd = false;
            return true;
        }
        return false;
    };
    
    
    // One off helper actions
    public static final Consumer<RobotAutoState> resetDrivetrain = autoState -> autoState.caiden.resetDrivetrain();
    
    //static public Consumer<RobotAutoState>
    public static final Consumer<RobotAutoState> actionRightRangeSensor = autoState -> {
        autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(autoState.caiden.getDistance(), autoState.distanceToWall), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                            -autoState.power - autoState.pivot,
                            -autoState.power + autoState.pivot,
                             autoState.power - autoState.pivot);
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0);
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
    };
    
    // Cone auto
    public static final Stage<RobotAutoState> goRightToWall = new Stage<>(actionRightRangeSensor,
    autoState -> {autoState.caiden.resetDrivetrain(); autoState.distanceToWall = 19;},
    resetDrivetrain, 
    rangeControllerIsEndPredicate);

    public static final Consumer<RobotAutoState> actionGoForwardToPosition = autoState -> {
        
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0.06);
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
    };
    
    
    public static final Stage<RobotAutoState> goForwardToConeStack = new Stage<>( actionGoForwardToPosition,
                                                    autoState -> autoState.distance = 1700,
                                                    forwardControllerIsEndPredicate);

    public static final Stage<RobotAutoState> goForwardToConeStackWithStartingCone = new Stage<>( autoState -> {
        
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0);
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
    }, 
    autoState -> autoState.distance = 1700,
    forwardControllerIsEndPredicate);


    public static final Stage<RobotAutoState> lineUpWithConeStack = new Stage<>( actionGoForwardToPosition,
                                                    null,
                                                    resetDrivetrain,
                                                    forwardControllerIsEndPredicate);
                                                    
    private static final int DISTANCE_TO_CONE = 9;
    public static final Stage<RobotAutoState> approachConeStack = new Stage<>(autoState -> {
            autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(autoState.caiden.getDistance(), autoState.distanceToWall), -0.6, 0.6);
            autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
            autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                                -autoState.power - autoState.pivot,
                                -autoState.power + autoState.pivot,
                                 autoState.power - autoState.pivot);
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
            autoState.caiden.armPosition(0.05);
            autoState.caiden.lazyR();
            autoState.caiden.openClaw();
        },
        autoState -> {autoState.caiden.resetDrivetrain(); autoState.distanceToWall = DISTANCE_TO_CONE;},
        autoState -> (rangeControllerIsEndPredicate.test(autoState) && autoState.caiden.elevatorIsInPosition())
        );

    public static final Stage<RobotAutoState> grabCone = new Stage<>(autoState -> {
            autoState.power = Range.clip(RobotAutoState.rangeSensorController.calculate(autoState.caiden.getDistance(), autoState.distanceToWall), -0.6, 0.6);
            autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
            autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                                -autoState.power - autoState.pivot,
                                -autoState.power + autoState.pivot,
                                 autoState.power - autoState.pivot);
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
            if(autoState.stageElapsedTime.milliseconds() < 400) {
                autoState.caiden.openClaw();
            } else {
                autoState.caiden.closeClaw();
            }
            autoState.caiden.armPosition(0.24);
            autoState.caiden.lazyR();
        },
        autoState -> {autoState.caiden.resetDrivetrain(); autoState.distanceToWall = DISTANCE_TO_CONE;},
        autoState -> autoState.stageElapsedTime.milliseconds() > 1200
    );

    public static final Stage<RobotAutoState> liftElevatorToClearConeStack = new Stage<>(autoState -> {
            autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
            autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                                -autoState.power - autoState.pivot,
                                -autoState.power + autoState.pivot,
                                 autoState.power - autoState.pivot);
            
            // 915 is low goal
            autoState.caiden.goToElevatorPosition(915);
            autoState.caiden.armPosition(0.28);
            autoState.caiden.lazyR();
            autoState.caiden.closeClaw();
        },
        autoState -> autoState.caiden.resetDrivetrain(),
        autoState -> autoState.stageElapsedTime.milliseconds() > 800
    );

    public static final Consumer<RobotAutoState> actionBackupToShortPole = autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.35, 0.35);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0.24);
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
        };

    public static final Stage<RobotAutoState> backupToShortPoleWithStartingCone = new Stage<>(actionBackupToShortPole,
        autoState -> {autoState.caiden.resetDrivetrain(); autoState.distance = -500;},
        forwardControllerIsEndPredicate
    );

    public static final Stage<RobotAutoState> backupToShortPole = new Stage<>(actionBackupToShortPole,
        autoState -> {autoState.caiden.resetDrivetrain(); autoState.distance = -480;},
        forwardControllerIsEndPredicate
    );
    
    public static final Consumer<RobotAutoState> actionLowerConeOntoPole = autoState -> {
        if(autoState.stageElapsedTime.milliseconds() > 300) {
            autoState.caiden.goToElevatorPosition(300);
        }
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);

        autoState.caiden.armPosition(0.24);
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
        };

    public static final Stage<RobotAutoState> lowerStartingConeOntoPole = new Stage<>(actionLowerConeOntoPole,
        autoState -> autoState.stageElapsedTime.milliseconds() > 400
    );
    
    public static final Stage<RobotAutoState> lowerConeOntoPole = new Stage<>(actionLowerConeOntoPole,
        autoState -> autoState.stageElapsedTime.milliseconds() > 600
    );
    
    public static final Consumer<RobotAutoState> actionOpenClaw = autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(300);
        autoState.caiden.armPosition(0.24);
        autoState.caiden.lazyL();
        autoState.caiden.openClaw();
        };

    public static final Stage<RobotAutoState> openClawWithStartingCone = new Stage<>(actionOpenClaw,
        autoState -> autoState.stageElapsedTime.milliseconds() > 400
    );

    public static final Stage<RobotAutoState> openClaw = new Stage<>(actionOpenClaw,
        autoState -> autoState.stageElapsedTime.milliseconds() > 400
    );

    public static final Stage<RobotAutoState> clearLowGoal = new Stage<>(autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.6, 0.6);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0.0);
        autoState.caiden.lazyR();
        autoState.caiden.openClaw();
        },
        autoState -> autoState.stageElapsedTime.milliseconds() > 700
    );
    
    public static final Stage<RobotAutoState> goBackToConeStack = new Stage<>(autoState -> {
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.3, 0.3);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0.0);
        autoState.caiden.lazyR();
        autoState.caiden.openClaw();
        },
        autoState -> {autoState.currentCone++; autoState.distance = 0;},
        forwardControllerIsEndPredicate
    );


    public static boolean seeingConeLine() {
        return state.caiden.colorSensorRed() > 500 || state.caiden.colorSensorBlue() > 500;
    }

    public static final Stage<RobotAutoState> testElevator = new Stage<>(autoState -> autoState.caiden.goToElevatorPosition(915),
    autoState -> false);


    public static final Stage<RobotAutoState> closeClawOnPreloadCone = new Stage<>(autoState -> {
        autoState.caiden.closeClaw();
        if(autoState.stageElapsedTime.milliseconds() > 1000) {
            autoState.caiden.goToElevatorPosition(915);
        }
    }, 
    autoState -> autoState.stageElapsedTime.milliseconds() > 1400);


    public static final Stage<RobotAutoState> findConeLinePosition = new Stage<>(autoState -> {
        if(seeingConeLine() && !autoState.seenConeLine) {
            autoState.seenConeLine = true;
        } else if(autoState.seenConeLine && !seeingConeLine()) {
            autoState.distance = autoState.caiden.getFRMotorCount() + 80;
            autoState.shouldEnd = true;
        }
    
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot,
                                       autoState.power + autoState.pivot,
                                       autoState.power - autoState.pivot);
                                       
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.armPosition(0);
        autoState.caiden.lazyL();
        autoState.caiden.closeClaw();
        
    }, elapsed -> state.power = 0.2, robotStateIsEndPredicate);
    
    
    // Should have 1-2 second timeout
    public static final Stage<RobotAutoState> recognizeSignalWithTimeout = new Stage<>(autoState -> {
        
        
        if(autoState.caiden.Slidey.getCurrentPosition() > 120) {
            autoState.caiden.lazyL();
        }
        autoState.caiden.goToElevatorPosition(915);
        autoState.caiden.closeClaw();

        if(autoState.stageElapsedTime.milliseconds() < 300) {
            return;
        }

        List<Recognition> recognizedObjects = autoState.vision.lookForObject();

        // step through the list of recognitions and display image position/size information for each one
        // Note: "Image number" refers to the randomized image orientation/number
        for (Recognition recognition : recognizedObjects) {
            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

            autoState.telemetry.addData(""," ");
            autoState.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
            autoState.telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
            autoState.telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
        }
        if(!recognizedObjects.isEmpty()) {
            autoState.recognizedSignal = recognizedObjects.stream().max(Comparator.comparingDouble(Recognition::getConfidence)).get().getLabel();
        }
    }, autoState -> {
        autoState.caiden.enableHeadlight();
        autoState.caiden.setHeadlightPower(0.06);
    }, autoState -> {
        autoState.caiden.disableHeadlight();
        autoState.caiden.setHeadlightPower(0.0);
    },
    recognizeSignalIsEndPredicate);
    
    

    public static final Stage<RobotAutoState> goToZone = new Stage<> (autoState -> {
    }, autoState -> true);


    public static final Stage<RobotAutoState> goToZone1 = new Stage<> (autoState -> {
        autoState.power = Range.clip(RobotAutoState.strafeController.calculate(autoState.caiden.getFRMotorCount(), 2400), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.armPosition(0.1);
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
        if(autoState.stageElapsedTime.milliseconds() > 2000) {
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        }
    },
    autoState -> autoState.caiden.reset(), autoState -> autoState.caiden.reset(), strafeControllerIsEndPredicate);

    public static final Stage<RobotAutoState> goToZone2 = new Stage<> (autoState -> {
        autoState.power = Range.clip(RobotAutoState.strafeController.calculate(autoState.caiden.getFRMotorCount(), 1200), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.armPosition(0.1);
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
        if(autoState.stageElapsedTime.milliseconds() > 2000) {
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        }
    },
    autoState -> autoState.caiden.reset(), strafeControllerIsEndPredicate);

    public static final Stage<RobotAutoState> goToZone3 = new Stage<> (autoState -> {
        autoState.power = 0;
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPower(autoState.power + autoState.pivot,
                -autoState.power - autoState.pivot,
                -autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.armPosition(0.1);
        autoState.caiden.lazyS();
        autoState.caiden.closeClaw();
        if(autoState.stageElapsedTime.milliseconds() > 2000) {
            autoState.caiden.goToElevatorPosition(autoState.coneHeight[autoState.currentCone]);
        }
    },
    autoState -> autoState.caiden.reset(), strafeControllerIsEndPredicate);

    public static final Stage<RobotAutoState> backupIntoZoneSlightly = new Stage<> (autoState -> {
        autoState.telemetry.addData("Backing up distance: ", autoState.distance);
        autoState.distance = -300;
        autoState.power = Range.clip(RobotAutoState.forwardController.calculate(autoState.caiden.getFRMotorCount(), autoState.distance), -0.7, 0.7);
        autoState.pivot = Range.clip(RobotAutoState.anglePID.calculate(autoState.caiden.getCachedHeading(), autoState.heading), -0.5, 0.5);
        autoState.caiden.driveRawPowerInAuto(autoState.power + autoState.pivot,
                autoState.power - autoState.pivot,
                autoState.power + autoState.pivot,
                autoState.power - autoState.pivot);
        autoState.caiden.goToElevatorPosition(0);
        autoState.caiden.armPosition(0.1);
    },
    autoState -> {autoState.caiden.resetDrivetrain(); autoState.caiden.reset(); autoState.distance = -300;}, null);

    
}

