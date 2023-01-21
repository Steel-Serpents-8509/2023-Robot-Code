package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Autonomous(name = "DefaultModelAuto")
public class TheRealAuto_Copy extends LinearOpMode {

    
    private CustomVision vision;
    private String recognizedSignal; 
    private CaidenRobot caiden;
 
    
     @Override 
     public void runOpMode() {
         
        vision = new CustomVision(hardwareMap);
        caiden = new CaidenRobot(hardwareMap);
      
          // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            postTelemetry();
        }
     
        waitForStart();
        caiden.reset();
        
        // Find zone
        //recognizeSignalZone();
        
        // Go to zone
        //goToZone();
        caiden.Claw.setPosition(0);
        while (caiden.Pot.getVoltage()>.65) {
            caiden.moveArm(1);
        }
        caiden.goToForwardPosition(2200);
     }
     
     public void goToZone() {
        telemetry.addData("Recognized Signal", recognizedSignal);
        postTelemetry();
        switch(recognizedSignal) {
            case "nut":
                goToZone2();
                break;
            case "snake":
                goToZone1();
                break;
            case "sohum":
                goToZone3();
                break;
            default:
                telemetry.addData("Error", "Signal not accounted for :(");
                    
        }
    }
     
    // strafe left, move forward
    private void goToZone1() {
        caiden.goToStrafePosition(2300);
        while(caiden.isBusy()){
            postTelemetry();
            idle();  
        }
        caiden.reset();
        
        caiden.goToForwardPosition(2200);
        while(caiden.isBusy()){
            postTelemetry();
            idle();  
        }
        caiden.reset();
    }
    
    // move forward
    private void goToZone2() {
        caiden.goToForwardPosition(2200);
        while(caiden.isBusy()){
            postTelemetry();
            idle();  
        }
        caiden.reset();
        
    }
    
    // strafe right, move forward
    private void goToZone3() {
        caiden.goToStrafePosition(-2300);
        while(caiden.isBusy()){
            postTelemetry();
            idle();  
        }
        caiden.reset();
        
        caiden.goToForwardPosition(2200);
        while(caiden.isBusy()){
            postTelemetry();
            idle();  
        }
        caiden.reset();
    }
   
    public void recognizeSignalZone() {
        while (opModeIsActive()) {
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
    
    
  //  @Override
    public void postTelemetry() {
        
        caiden.updateTelemetry(telemetry);
        //telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        vision.updateTelemetry(telemetry);
        telemetry.update();
    }
    
    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    /*public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }*/

    /**
     * Reset the "offset" heading back to zero
     */
    /*public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
        
    }*/
}
