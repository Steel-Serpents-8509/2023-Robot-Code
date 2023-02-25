package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.CaidenRobot;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Autonomous
@Disabled
public class TheFakeAuto extends LinearOpMode {

    CaidenRobot caiden;
    
    private CustomVision vision;
    PIDController forwardController = new PIDController(0.0022, 0.044, 0.00034);
    PIDController strafeController = new PIDController(0.01, 0.025, 0.0);
    PIDController rangeSensorController = new PIDController(0.05, 0.092, 0.0035);
    PIDController anglePID = new PIDController(0.024, 0.02, 0.00);
    /*PIDController forwardController = new PIDController(0.0016, 0.00, 0.0);
    PIDController strafeController = new PIDController(0.01, 0.02, 0.0);
    PIDController rangeSensorController = new PIDController(0.04, 0.017, 0.0);
    PIDController anglePID = new PIDController(0.012, 0.02, 0.001);*/
    int heading = 0;
    int autoStage = -1;
    boolean robotStopped = false;
    long robotStoppedTimestamp = 0;
    boolean ranOnce = false;
    double power;
    double pivot;
    ElapsedTime elapsedTime;
    
    private static double DIST_TO_WALL = 14.8;
    private static int ELEVATOR_TOP = 830;
    private String recognizedSignal = "square"; 
    
    //                          cone5,  cone4, etc
    int coneHeight[] =       {145,  140, 105, 70,  35};
    //double robotDistance[] =    {25, 23,    21,  14.2,  12.4}; 
    int currentCone = 0;
    int maxCone = 1;
    
    @Override
    public void runOpMode() {
        
        caiden = new CaidenRobot(hardwareMap);
        caiden.setHeadlightPower(0.14);
        
        vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/black_shapes_good_videos.tflite");
        //vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/original_sleeve_good_data.tflite");
        forwardController.setTolerance(20);
        strafeController.setTolerance(20);
        rangeSensorController.setTolerance(1.3);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        
        recognizeSignalZone();
        caiden.setHeadlightPower(0);
        

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Go forward
            if(autoStage == -1) {
                power = Range.clip(forwardController.calculate(caiden.getFRMotorCount(), 2600), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, power - pivot, power + pivot, power - pivot);

                caiden.openClaw();
                
                if(forwardController.atSetPoint()) {
                    caiden.lazyR();
                }
                
                if(forwardController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!forwardController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(forwardController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 100000000L)) {
                    //caiden.stop();
                    robotStopped = false;
                    autoStage = 0;
                    robotStoppedTimestamp = System.nanoTime();
                }
                // Go slightly back
            } if(autoStage == 0) {
                power = Range.clip(forwardController.calculate(caiden.getFRMotorCount(), 2180), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, power - pivot, power + pivot, power - pivot);

                caiden.openClaw();
                
                if(forwardController.atSetPoint()) {
                    caiden.lazyR();
                }
                
                if(forwardController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!forwardController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(forwardController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 500000000L)) {
                    //caiden.stop();
                    robotStopped = false;
                    autoStage = 1;
                }
                // Go towards the cones
            } else if(autoStage == 1) {
                power = Range.clip(rangeSensorController.calculate(caiden.getDistance(), DIST_TO_WALL), -0.6, 0.6);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, -power - pivot, -power + pivot, power - pivot);
                caiden.goToElevatorPosition(coneHeight[currentCone]);

                caiden.lazyR();
                caiden.openClaw();
                if(rangeSensorController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!rangeSensorController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(rangeSensorController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 200000000L)) {
                    //caiden.stop();
                    robotStopped = false;
                    autoStage = 2;
                    robotStoppedTimestamp = System.nanoTime();
                }
                // Close claw
            } else if(autoStage == 2) {
                power = Range.clip(rangeSensorController.calculate(caiden.getDistance(), DIST_TO_WALL), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, -power - pivot, -power + pivot, power - pivot);
                caiden.goToElevatorPosition(coneHeight[currentCone]);
                caiden.lazyR();
                caiden.closeClaw();
                if(((System.nanoTime() - robotStoppedTimestamp) > 1000000000L)) {
                    //caiden.stop();
                    autoStage = 3;
                    ranOnce = false;
                    robotStoppedTimestamp = System.nanoTime();
                }
                // Lift elevator
            } else if(autoStage == 3) {
                if(!ranOnce) {
                    ranOnce = true;
                    caiden.lazyR();
                    caiden.closeClaw();
                }
                
                caiden.goToElevatorPosition(400);
                //caiden.armPosition(1);
                power = Range.clip(rangeSensorController.calculate(caiden.getDistance(), DIST_TO_WALL), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, -power - pivot, -power + pivot, power - pivot);
                if(((System.nanoTime() - robotStoppedTimestamp) > 800000000L)) {
                    //caiden.stop();
                    caiden.reset();
                    robotStopped = false;
                    ranOnce = false;
                    autoStage = 4;
                }
                // Go towards pole
            } else if(autoStage == 4) {
                if(!ranOnce) {
                    ranOnce = true;
                    caiden.lazyS();
                    caiden.closeClaw();
                    //caiden.reset();
                }
                
                
                caiden.goToElevatorPosition(ELEVATOR_TOP);

                //power = 0;
                //power = strafeController.calculate(caiden.getDistance(), 2000);
                //power = Range.clip(strafeController.calculate(caiden.getFRMotorCount(), 1830), -0.8, 0.8);
                power = Range.clip(strafeController.calculate(caiden.getFRMotorCount(), 1800), -0.8, 0.8);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPowerInAuto(power + pivot, -power - pivot, -power + pivot, power - pivot);
                //caiden.driveRawPowerInAuto(pivot, -pivot, pivot, -pivot);
                if(strafeController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!strafeController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(strafeController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 1000000000L)) {
                    caiden.stop();
                    robotStopped = false;
                    autoStage = 5;
                    robotStoppedTimestamp = System.nanoTime();
                    ranOnce = false;
                }
            } else if(autoStage == 5) {
                if(!ranOnce) {
                    ranOnce = true;
                }
                

                if(((System.nanoTime() - robotStoppedTimestamp) > 1000000000L)) {
                    caiden.reset();
                    robotStopped = false;
                    
                    robotStoppedTimestamp = System.nanoTime();
                    ranOnce = false;
                    autoStage = 6;
                }
            } else if(autoStage == 6) {
                if(!ranOnce) {
                    ranOnce = true;
                    caiden.openClaw();
                    currentCone++;
                    if(currentCone >= maxCone) {
                        caiden.stop();
                        while(opModeIsActive()) {
                            goToZone();
                        }
                    }
                }
                if(((System.nanoTime() - robotStoppedTimestamp) > 1000000000L)) {
                    caiden.reset();
                    robotStopped = false;
                    ranOnce = false;
                    robotStoppedTimestamp = System.nanoTime();
                    autoStage = 1;
                }
            }
            
            
            updateTelemetry();
            

        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("RobotStopped", robotStopped);
        telemetry.addData("RobotStoppedTime", robotStoppedTimestamp);
        telemetry.addData("AutoStage", autoStage);
        telemetry.addData("Power", power);
        telemetry.addData("Pivot", pivot);
        telemetry.addData("Recognized Signal", recognizedSignal);
        caiden.updateTelemetry(telemetry);
        telemetry.update();
    }
    
       
    public void recognizeSignalZone() {
        elapsedTime = new ElapsedTime();
        
        while (opModeIsActive() && (elapsedTime.milliseconds() < 1000)) {
            List<Recognition> recognizedObjects = vision.lookForObject();
            telemetry.addData("# Objects Detected", recognizedObjects.size());
            
            // step through the list of recognitions and display image position/size information for each one
            // Note: "Image number" refers to the randomized image orientation/number
            for (Recognition recognition : recognizedObjects) {
                double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
            }
            if(recognizedObjects.size() > 0) {
                recognizedSignal = recognizedObjects.get(0).getLabel();
                break;
            }
            
            postTelemetry();
        }
    }
    
    
    public void postTelemetry() {
        
        caiden.updateTelemetry(telemetry);
        //telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        if(vision != null) {
            vision.updateTelemetry(telemetry);
        }
        telemetry.update();
    }
    
    public void goToZone() {
        caiden.reset();
        autoStage = -1;
        switch(recognizedSignal) {
            case "none": 
                break;
            case "square":
                goToZone2();
                break;
            case "circle":
                goToZone3();
                break;
            case "triangle":
                goToZone1();
                break;
            default:
                telemetry.addData("Error", "Signal not accounted for :(");
        }
    }
    // strafe left, move forward
    private void goToZone1() {
        while(opModeIsActive()) {
            if(autoStage == -1) {
                
            power = Range.clip(strafeController.calculate(caiden.getFRMotorCount(), -1900), -0.8, 0.8);
            pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
            caiden.driveRawPowerInAuto(power + pivot, -power - pivot, -power + pivot, power - pivot);
            caiden.goToElevatorPosition(0);

            updateTelemetry();
                if(strafeController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!strafeController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(strafeController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 100000000L)) {
                    caiden.stop();
                    
                    caiden.reset();
                    robotStopped = false;
                    autoStage = 0;
                    robotStoppedTimestamp = System.nanoTime();
                }
                // Go slightly back
            } if(autoStage == 0) {
                power = Range.clip(forwardController.calculate(caiden.getFRMotorCount(), -300), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, power - pivot, power + pivot, power - pivot);

                caiden.goToElevatorPosition(0);
                
                /*if(forwardController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!forwardController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(forwardController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 500000000L)) {
                    caiden.stop();
                    robotStopped = false;
                    autoStage = 1;
                }*/
                // Go towards the cones
            } 
        }
    }
    
    // move forward
    private void goToZone2() {
        while(opModeIsActive()) {
            if(autoStage == -1) {
                
            power = Range.clip(strafeController.calculate(caiden.getFRMotorCount(), -800), -0.8, 0.8);
            pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
            caiden.driveRawPowerInAuto(power + pivot, -power - pivot, -power + pivot, power - pivot);
            caiden.goToElevatorPosition(0);

            updateTelemetry();
                
                /*if(strafeController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!strafeController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(strafeController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 100000000L)) {
                    caiden.stop();
                    
                    caiden.reset();
                    robotStopped = false;
                    autoStage = 0;
                    robotStoppedTimestamp = System.nanoTime();
                }*/
                // Go slightly back
            } if(autoStage == 0) {
                power = Range.clip(forwardController.calculate(caiden.getFRMotorCount(), -300), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, power - pivot, power + pivot, power - pivot);
                
                caiden.goToElevatorPosition(0);
                if(forwardController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!forwardController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(forwardController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 500000000L)) {
                    caiden.stop();
                    robotStopped = false;
                    autoStage = 1;
                }
                // Go towards the cones
            } 
        }
        
    }
    
    // strafe right, move forward
    private void goToZone3() {
        while(opModeIsActive()) {
            if(autoStage == -1) {
                
            power = Range.clip(strafeController.calculate(caiden.getFRMotorCount(), 700), -0.8, 0.8);
            pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
            caiden.driveRawPowerInAuto(power + pivot, -power - pivot, -power + pivot, power - pivot);
            caiden.goToElevatorPosition(0);

            updateTelemetry();
                
                if(strafeController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!strafeController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(strafeController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 100000000L)) {
                    caiden.stop();
                    caiden.reset();
                    robotStopped = false;
                    autoStage = 0;
                    robotStoppedTimestamp = System.nanoTime();
                }
                // Go slightly back
            } if(autoStage == 0) {
                power = Range.clip(forwardController.calculate(caiden.getFRMotorCount(), -300), -0.7, 0.7);
                pivot = anglePID.calculate(caiden.getCachedHeading(), heading);
                caiden.driveRawPower(power + pivot, power - pivot, power + pivot, power - pivot);
                caiden.goToElevatorPosition(0);
                /*if(forwardController.atSetPoint() && !robotStopped) {
                    robotStoppedTimestamp = System.nanoTime();
                    robotStopped = true;
                } else if(!forwardController.atSetPoint() && robotStopped) {
                    robotStopped = false;
                } else if(forwardController.atSetPoint() && robotStopped && (System.nanoTime() - robotStoppedTimestamp > 500000000L)) {
                    caiden.stop();
                    robotStopped = false;
                    autoStage = 1;
                }*/
                // Go towards the cones
            } 
        }
    }
}
    


