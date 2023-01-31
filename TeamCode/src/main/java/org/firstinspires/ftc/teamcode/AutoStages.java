package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.AutoSequencer.*;

import java.util.Comparator;
import java.util.function.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CustomVision;
import java.util.List;

public class AutoStages {
    
    public static RobotAutoState state = new RobotAutoState();
    public static final AutoSequencer<RobotAutoState> sequencer = new AutoSequencer<>(state);

    
    /*
     *  
     *  Predicates:
     *  Conditions to leaving the stage
     * 
     */
    static public Predicate<RobotAutoState> recognizeSignalIsEndPredicate = (state) -> {
        return state.recognizedSignal != "" || state.stageElapsedTime.milliseconds() > 2500;
    };
    
    static public Predicate<RobotAutoState> forwardControllerIsEndPredicate = (state) -> {
        if(RobotAutoState.forwardController.atSetPoint() && state.anglePID.atSetPoint() && !state.robotInPosition) {
            state.stageElapsedTime.reset();
            state.robotInPosition = true;
        } else if(!RobotAutoState.forwardController.atSetPoint() && state.robotInPosition) {
            state.robotInPosition = false;
        } else if(state.forwardController.atSetPoint() && state.robotInPosition && state.stageElapsedTime.milliseconds() > 400) {
            return true;
        }
        return false;
    };
    static public Predicate<RobotAutoState> strafeControllerIsEndPredicate = (state) -> {
        if(RobotAutoState.strafeController.atSetPoint() && !state.robotInPosition) {
            state.stageElapsedTime.reset();
            state.robotInPosition = true;
        } else if(!RobotAutoState.strafeController.atSetPoint() && state.robotInPosition) {
            state.robotInPosition = false;
        } else if(state.strafeController.atSetPoint() && state.robotInPosition && state.stageElapsedTime.milliseconds() > 400) {
            return true;
        }
        return false;
    };
    static public Predicate<RobotAutoState> rangeControllerIsEndPredicate = (state) -> {
        if(RobotAutoState.rangeSensorController.atSetPoint() && state.anglePID.atSetPoint() && !state.robotInPosition) {
            state.stageElapsedTime.reset();
            state.robotInPosition = true;
        } else if(!RobotAutoState.rangeSensorController.atSetPoint() && state.robotInPosition) {
            state.robotInPosition = false;
        } else if(state.rangeSensorController.atSetPoint() && state.robotInPosition && state.stageElapsedTime.milliseconds() > 400) {
            return true;
        }
        return false;
    };
    
    static public Predicate<RobotAutoState> robotStateIsEndPredicate = (state) -> {
        if(state.shouldEnd) {
            state.shouldEnd = false;
            return true;
        }
        return false;
    };
    
    
    // One off helper actions
    static public Consumer<RobotAutoState> resetDrivetrain = (state) -> {state.caiden.resetDrivetrain();};
    
    //static public Consumer<RobotAutoState>
    static public Consumer<RobotAutoState> actionRightRangeSensor = (state) -> {
        state.power = Range.clip(RobotAutoState.rangeSensorController.calculate(state.caiden.getDistance(), state.distanceToWall), -0.7, 0.7);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPower(state.power + state.pivot, 
                            -state.power - state.pivot, 
                            -state.power + state.pivot, 
                             state.power - state.pivot);
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0);
        state.caiden.lazyL();
        state.caiden.closeClaw();
    };
    
    // Cone auto
    static public Stage<RobotAutoState> goRightToWall = new Stage<>(actionRightRangeSensor,
    (state) -> {state.caiden.resetDrivetrain(); state.distanceToWall = 19;},
    resetDrivetrain, 
    rangeControllerIsEndPredicate);
    
    static public Consumer<RobotAutoState> actionGoForwardToPosition = (state) -> {
        
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.7, 0.7);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0.06);
        state.caiden.lazyL();
        state.caiden.closeClaw();
    };
    
    
    static public Stage<RobotAutoState> goForwardToConeStack = new Stage<>( actionGoForwardToPosition, 
                                                    (state) -> {state.distance = 1700;}, 
                                                    forwardControllerIsEndPredicate);
    
    static public Stage<RobotAutoState> goForwardToConeStackWithStartingCone = new Stage<>( (state) -> {
        
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.7, 0.7);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0);
        state.caiden.lazyL();
        state.caiden.closeClaw();
    }, 
    (state) -> {state.distance = 1700;},
    forwardControllerIsEndPredicate);
                                                    
                                                    
                                                    
    static public Stage<RobotAutoState> lineUpWithConeStack = new Stage<>( actionGoForwardToPosition, 
                                                    null,
                                                    resetDrivetrain,
                                                    forwardControllerIsEndPredicate);
                                                    
    private static int distanceToCone = 9;
    static public Stage<RobotAutoState> approachConeStack = new Stage<>((state) -> {
            state.power = Range.clip(RobotAutoState.rangeSensorController.calculate(state.caiden.getDistance(), state.distanceToWall), -0.6, 0.6);
            state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
            state.caiden.driveRawPower(state.power + state.pivot, 
                                -state.power - state.pivot, 
                                -state.power + state.pivot, 
                                 state.power - state.pivot);
            state.caiden.goToElevatorPosition(state.coneHeight[state.currentCone]);
            state.caiden.armPosition(0.05);
            state.caiden.lazyR();
            state.caiden.openClaw();
        },
        (state) -> {state.caiden.resetDrivetrain(); state.distanceToWall = distanceToCone;},
        (state) -> (rangeControllerIsEndPredicate.test(state) && state.caiden.elevatorIsInPosition())
        );
    
    static public Stage<RobotAutoState> grabCone = new Stage<>((state) -> {
            state.power = Range.clip(RobotAutoState.rangeSensorController.calculate(state.caiden.getDistance(), state.distanceToWall), -0.6, 0.6);
            state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
            state.caiden.driveRawPower(state.power + state.pivot, 
                                -state.power - state.pivot, 
                                -state.power + state.pivot, 
                                 state.power - state.pivot);
            state.caiden.goToElevatorPosition(state.coneHeight[state.currentCone]);
            if(state.stageElapsedTime.milliseconds() < 400) {
                state.caiden.openClaw();
            } else {
                state.caiden.closeClaw();
            }
            state.caiden.armPosition(0.24);
            state.caiden.lazyR();
        },
        (state) -> {state.caiden.resetDrivetrain(); state.distanceToWall = distanceToCone;},
        (state) -> {return state.stageElapsedTime.milliseconds() > 1200;}
    );
    
    static public Stage<RobotAutoState> liftElevatorToClearConeStack = new Stage<>((state) -> {
            //state.power = Range.clip(rangeSensorController.calculate(state.caiden.getDistance(), state.distanceToWall), -0.6, 0.6);
            state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
            state.caiden.driveRawPower(state.power + state.pivot, 
                                -state.power - state.pivot, 
                                -state.power + state.pivot, 
                                 state.power - state.pivot);
            
            // 915 is low goal
            state.caiden.goToElevatorPosition(915);
            state.caiden.armPosition(0.28);
            state.caiden.lazyR();
            state.caiden.closeClaw();
        },
        (state) -> {state.caiden.resetDrivetrain();},
        (state) -> {return state.stageElapsedTime.milliseconds() > 800;}
    );
    
    static public Consumer<RobotAutoState> actionBackupToShortPole = (state) -> {
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.35, 0.35);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0.24);
        state.caiden.lazyL();
        state.caiden.closeClaw();
        };
    
    static public Stage<RobotAutoState> backupToShortPoleWithStartingCone = new Stage<>(actionBackupToShortPole,
        (state) -> {state.caiden.resetDrivetrain(); state.distance = -500;},
        forwardControllerIsEndPredicate
    );
    
    static public Stage<RobotAutoState> backupToShortPole = new Stage<>(actionBackupToShortPole,
        (state) -> {state.caiden.resetDrivetrain(); state.distance = -480;},
        forwardControllerIsEndPredicate
    );
    
    public static Consumer<RobotAutoState> actionLowerConeOntoPole = (state) -> {
        if(state.stageElapsedTime.milliseconds() > 300) {
            state.caiden.goToElevatorPosition(300);
        }
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.6, 0.6);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);

        state.caiden.armPosition(0.24);
        state.caiden.lazyL();
        state.caiden.closeClaw();
        };
    
    static public Stage<RobotAutoState> lowerStartingConeOntoPole = new Stage<>(actionLowerConeOntoPole,
        (state) -> {return state.stageElapsedTime.milliseconds() > 400;}
    );
    
    static public Stage<RobotAutoState> lowerConeOntoPole = new Stage<>(actionLowerConeOntoPole,
        (state) -> {return state.stageElapsedTime.milliseconds() > 600;}
    );
    
    public static Consumer<RobotAutoState> actionOpenClaw = (state) -> {
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.6, 0.6);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(300);
        state.caiden.armPosition(0.24);
        state.caiden.lazyL();
        state.caiden.openClaw();
        };
    
    static public Stage<RobotAutoState> openClawWithStartingCone = new Stage<>(actionOpenClaw,
        (state) -> {return state.stageElapsedTime.milliseconds() > 400;}
    );
    
    static public Stage<RobotAutoState> openClaw = new Stage<>(actionOpenClaw,
        (state) -> {return state.stageElapsedTime.milliseconds() > 400;}
    );
    
    static public Stage<RobotAutoState> clearLowGoal = new Stage<>((state) -> {
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.6, 0.6);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0.0);
        state.caiden.lazyR();
        state.caiden.openClaw();
        },
        (state) -> {return state.stageElapsedTime.milliseconds() > 700;}
    );
    
    static public Stage<RobotAutoState> goBackToConeStack = new Stage<>((state) -> {
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.3, 0.3);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0.0);
        state.caiden.lazyR();
        state.caiden.openClaw();
        },
        (state) -> {state.currentCone++; state.distance = 0;},
        forwardControllerIsEndPredicate
    );
    
    
    static public boolean seeingConeLine() {
        return state.caiden.colorSensorRed() > 500 || state.caiden.colorSensorBlue() > 500;
    }
    
    static public Stage<RobotAutoState> testElevator = new Stage<>((state) -> {
        state.caiden.goToElevatorPosition(915);
    }, 
    (state) -> {
        return false;
    });
   
    
    static public Stage<RobotAutoState> closeClawOnPreloadCone = new Stage<>((state) -> {
        state.caiden.closeClaw();
        if(state.stageElapsedTime.milliseconds() > 1000) {
            state.caiden.goToElevatorPosition(915);
        }
    }, 
    (state) -> {
        return state.stageElapsedTime.milliseconds() > 1400;
    });
    
    
    static public Stage<RobotAutoState> findConeLinePosition = new Stage<>((state) -> {
        if(seeingConeLine() && !state.seenConeLine) {
            state.seenConeLine = true;
        } else if(state.seenConeLine && !seeingConeLine()) {
            state.distance = state.caiden.getFRMotorCount() + 80;
            state.shouldEnd = true;
        }
    
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot, 
                                       state.power - state.pivot, 
                                       state.power + state.pivot, 
                                       state.power - state.pivot);
                                       
        state.caiden.goToElevatorPosition(915);
        state.caiden.armPosition(0);
        state.caiden.lazyL();
        state.caiden.closeClaw();
        
    }, (elapsed) -> {state.power = 0.2;}, robotStateIsEndPredicate);
    
    
    // Should have 1-2 second timeout
    public static Stage<RobotAutoState> recognizeSignalWithTimeout = new Stage<>((state) -> {
        
        
        if(state.caiden.getElevatorPosition() > 120) {
            state.caiden.lazyL();
        }
        state.caiden.goToElevatorPosition(915);
        state.caiden.closeClaw();

        if(state.stageElapsedTime.milliseconds() < 300) {
            return;
        }

        List<Recognition> recognizedObjects = state.vision.lookForObject();

        //telemetry.addData("# Objects Detected", recognizedObjects.size());
        
        // step through the list of recognitions and display image position/size information for each one
        // Note: "Image number" refers to the randomized image orientation/number
        for (Recognition recognition : recognizedObjects) {
            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

            state.telemetry.addData(""," ");
            state.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
            state.telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
            state.telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
        }
        if(recognizedObjects.size() > 0) {
            state.recognizedSignal = recognizedObjects.stream().max(Comparator.comparingDouble(Recognition::getConfidence)).get().getLabel();
        }
    }, (state) -> {
        state.caiden.enableHeadlight();
        state.caiden.setHeadlightPower(0.06);
    }, (state) -> {
        state.caiden.disableHeadlight();
        state.caiden.setHeadlightPower(0.0);
    },
    //(state) -> {return false;});
    recognizeSignalIsEndPredicate);
    
    

    public static Stage<RobotAutoState> goToZone = new Stage<> ((state) -> {
    }, (state) -> {
        return true;
    });


    public static Stage<RobotAutoState> goToZone1 = new Stage<> ((state) -> {
        state.power = Range.clip(RobotAutoState.strafeController.calculate(state.caiden.getFRMotorCount(), 2400), -0.7, 0.7);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPower(state.power + state.pivot,
                -state.power - state.pivot,
                -state.power + state.pivot,
                state.power - state.pivot);
        state.caiden.armPosition(0.1);
        state.caiden.lazyS();
        state.caiden.closeClaw();
        if(state.stageElapsedTime.milliseconds() > 2000) {
            state.caiden.goToElevatorPosition(state.coneHeight[state.currentCone]);
        }
    },
    (state) -> {state.caiden.reset();}, (state)-> {state.caiden.reset();}, strafeControllerIsEndPredicate);

    public static Stage<RobotAutoState> goToZone2 = new Stage<> ((state) -> {
        state.power = Range.clip(RobotAutoState.strafeController.calculate(state.caiden.getFRMotorCount(), 1200), -0.7, 0.7);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPower(state.power + state.pivot,
                -state.power - state.pivot,
                -state.power + state.pivot,
                state.power - state.pivot);
        state.caiden.armPosition(0.1);
        state.caiden.lazyS();
        state.caiden.closeClaw();
        if(state.stageElapsedTime.milliseconds() > 2000) {
            state.caiden.goToElevatorPosition(state.coneHeight[state.currentCone]);
        }
    },
    (state) -> {state.caiden.reset();}, strafeControllerIsEndPredicate);

    public static Stage<RobotAutoState> goToZone3 = new Stage<> ((state) -> {
        state.power = 0;
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPower(state.power + state.pivot,
                -state.power - state.pivot,
                -state.power + state.pivot,
                state.power - state.pivot);
        state.caiden.armPosition(0.1);
        state.caiden.lazyS();
        state.caiden.closeClaw();
        if(state.stageElapsedTime.milliseconds() > 2000) {
            state.caiden.goToElevatorPosition(state.coneHeight[state.currentCone]);
        }
    },
    (state) -> {state.caiden.reset();}, strafeControllerIsEndPredicate);

    public static Stage<RobotAutoState> backupIntoZoneSlightly = new Stage<> ((state) -> {
        state.telemetry.addData("Backing up distance: ", state.distance);
        state.distance = -300;
        state.power = Range.clip(RobotAutoState.forwardController.calculate(state.caiden.getFRMotorCount(), state.distance), -0.7, 0.7);
        state.pivot = Range.clip(RobotAutoState.anglePID.calculate(state.caiden.getCachedHeading(), state.heading), -0.5, 0.5);
        state.caiden.driveRawPowerInAuto(state.power + state.pivot,
                state.power - state.pivot,
                state.power + state.pivot,
                state.power - state.pivot);
        state.caiden.goToElevatorPosition(0);
        state.caiden.armPosition(0.1);
    },
    (state) -> {state.caiden.resetDrivetrain(); state.caiden.reset(); state.distance = -300;}, null);

    
}

