package org.firstinspires.ftc.teamcode.competition;

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

import org.firstinspires.ftc.teamcode.AutoStages;
import org.firstinspires.ftc.teamcode.CustomVision;
import org.firstinspires.ftc.teamcode.CaidenRobot;

@Autonomous(name = "TheRealAuto")
public class TheRealAuto extends LinearOpMode {


    private CustomVision vision;
    private String recognizedSignal;
    private CaidenRobot caiden;
    private ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        caiden = new CaidenRobot(hardwareMap);
        vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_shape_model.tflite");
        //vision = new CustomVision(hardwareMap, "/sdcard/FIRST/tflitemodels/best_original_sleeve_model.tflite");
        //caiden.enableHeadlight();
        caiden.setHeadlightPower(0.1);




        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            postTelemetry();
        }



        waitForStart();
        caiden.reset();
        elapsedTime.reset();

        // Find zone
        recognizeSignalZone();

        //caiden.disableHeadlight();
        caiden.setHeadlightPower(0);

        // Go to zone
        goToZone();

    }

    public void goToZone() {
        telemetry.addData("Recognized Signal", recognizedSignal);
        postTelemetry();
        switch(recognizedSignal) {
            case "square":
            case "":
                goToZone2();
                break;
            case "circle":
                goToZone1();
                break;
            case "triangle":
                goToZone3();
                break;
            default:
                telemetry.addData("Error", "Signal not accounted for :(");
        }
    }

    // strafe left, move forward
    private void goToZone1() {
        caiden.goToStrafePosition(2350);
        while(caiden.isBusy()){
            caiden.goToElevatorPosition(915);
            postTelemetry();
            idle();
        }
        caiden.stop();
        caiden.reset();

        caiden.goToForwardPosition(2400);
        while(caiden.isBusy()){
            caiden.goToElevatorPosition(915);
            postTelemetry();
            idle();
        }
        caiden.stop();
        caiden.reset();

        while (opModeIsActive()) {
            caiden.goToElevatorPosition(0);
            postTelemetry();
        }


    }

    // move forward
    private void goToZone2() {
        caiden.goToForwardPosition(3000);
        while(caiden.isBusy()){
            caiden.goToElevatorPosition(915);
            postTelemetry();
            idle();
        }

        caiden.goToForwardPosition(2400);
        while(caiden.isBusy()){
            caiden.goToElevatorPosition(915);
            postTelemetry();
            idle();
        }
        caiden.stop();
        caiden.reset();

        while (opModeIsActive()) {
            caiden.goToElevatorPosition(0);
            postTelemetry();
        }
    }

    // strafe right, move forward
    private void goToZone3() {
        caiden.goToStrafePosition(-2350);
        while(caiden.isBusy()){
            caiden.goToElevatorPosition(915);
            postTelemetry();
            idle();
        }
        caiden.stop();
        caiden.reset();

        caiden.goToForwardPosition(2400);
        while(caiden.isBusy()) {
            caiden.goToElevatorPosition(915);
            postTelemetry();
            idle();
        }
        caiden.stop();
        caiden.reset();
        while (opModeIsActive()) {
            caiden.goToElevatorPosition(0);
            postTelemetry();
        }
    }

    public void recognizeSignalZone() {
        while (opModeIsActive() && elapsedTime.milliseconds() < 15000) {

            if(elapsedTime.milliseconds() < 1000) {
                caiden.closeClaw();
            } else {
                caiden.closeClaw();
                caiden.goToElevatorPosition(915);
            }

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

        telemetry.addData("Elapsed Time", elapsedTime.milliseconds());
        caiden.updateTelemetry(telemetry);
        //telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        if(vision != null) {
            vision.updateTelemetry(telemetry);
        }
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
