/*
Copyright 2023 FIRST Tech Challenge Team 8509

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.CaidenRobot;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.ButtonDebounce;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp
@Disabled
public class HuntersTeleop extends OpMode {
    /* Declare OpMode members. */

    CaidenRobot caiden;
    PIDController anglePID;
    
        double forward = 0;
        double horizontal = 0;
        double pivot = 0;
        int heading = 0;
        double headlightPower = 0;
        double shortHeight = 449;
        double medHeight = 751;
        double topHeight = 751;
        
        boolean strafe1;
        boolean strafe2;
        boolean raise;
        boolean lower;
        boolean raiseArm;
        boolean lowerArm;
        boolean close;
        boolean open;
        boolean left;
        boolean right;
        boolean straight;
        boolean saveRaise = true;
       // boolean quick;
        boolean first;
        boolean second;
        boolean third;
        boolean fourth;
        
        
        double headingDifference; 
        double headingDifferenceSign;
        
        boolean enablePID = true;
        
        ButtonDebounce rotateCW90 = new ButtonDebounce();
        ButtonDebounce rotateCCW90 = new ButtonDebounce();
    
    public void moveDriveTrain() {

        //forward = 0;
        //horizontal = 0;
        boolean currentY = gamepad1.y;
        boolean currentX = gamepad1.x;

        forward = -(gamepad1.left_stick_y * 0.7);
        horizontal = -(gamepad1.left_stick_x * 0.7);
        //pivot = -(gamepad1.left_stick_x * 0.6);
         
        //strafe1 = gamepad1.right_bumper;
        //strafe2 = gamepad1.left_bumper;
        
        raise = gamepad2.right_trigger > .1;
        lower = gamepad2.left_trigger > .1;
        
        raiseArm = gamepad2.left_stick_y > 0.5;
        lowerArm = gamepad2.left_stick_y < -0.5;
        
        first = gamepad2.dpad_down;
        second = gamepad2.dpad_left;
        third = gamepad2.dpad_up;
        fourth = gamepad2.dpad_right;
        
        //quick = gamepad2.a;
        
        close = gamepad1.right_trigger > .1;
        open = gamepad1.left_trigger > .1;
        
        left = gamepad2.b;
        straight = gamepad2.a;
        right = gamepad2.x;
        
        telemetry.addData("ForwardPower", forward);
        telemetry.addData("HorizontalPower", horizontal);
        //telemetry.addData("ForwardPower", vertical);
        if(rotateCCW90.getButton(gamepad1.left_bumper)){
            if (heading != 180){
                heading+=90;
            } else if(heading == 180) {
                heading = -90;
            }
            
        } else if (rotateCW90.getButton(gamepad1.right_bumper)){
            if(heading != -180){
                heading-=90;
            } else if(heading == -180) {
                heading = 90;
            }
        }
        if (left){
            caiden.lazyL();
        } else if (right){
            caiden.lazyR();
        } else if (straight){
            caiden.lazyS();
        }
        //robot stays straight
        //if(currentY) {
            //enablePID = false;
        //} 
        if(currentX) {
            enablePID = false;
        }
        
        if(enablePID) {
            double cachedHeading = Math.abs(caiden.getCachedHeading());
            double absHeading = Math.abs(heading);
            
            headingDifference = (caiden.getCachedHeading() - heading) % 360;
            
            if (Math.abs(headingDifference) > 180) {
                headingDifferenceSign = 360 - Math.abs(headingDifference);
                if (headingDifference > 0) {
                    headingDifference = -headingDifferenceSign; 
                } else {
                    headingDifference = headingDifferenceSign; 
                }
            }
            pivot = anglePID.calculate(headingDifference, 0);
        } else {
            pivot = 0;
        }

        /*double BRPower = (pivot + (-vertical + horizontal));
        double BLPower = (pivot + (-vertical - horizontal));
        double FRPower = (pivot + (-vertical + horizontal));
        double FLPower = (pivot + (-vertical - horizontal));*/
        
        double angle = caiden.getCachedHeading() * Math.PI/180;
        double powerMultiplier = gamepad1.a ? 0.5 : 1;
        //angle = 0; 
        
        double forwardCos = forward * Math.cos(angle);
        double forwardSin = -forward * Math.sin(angle);
        double horizontalSin = horizontal * Math.sin(angle);
        double horizontalCos = horizontal * Math.cos(angle);
        
        double fieldOrientedForward = forwardCos + horizontalSin;
        double fieldOrientedHorizontal = horizontalCos + forwardSin;
        
        double FRPower = (fieldOrientedForward + fieldOrientedHorizontal + pivot) * powerMultiplier;
        double FLPower = (fieldOrientedForward - fieldOrientedHorizontal - pivot) * powerMultiplier;
        double BRPower = (fieldOrientedForward - fieldOrientedHorizontal + pivot) * powerMultiplier;
        double BLPower = (fieldOrientedForward + fieldOrientedHorizontal - pivot) * powerMultiplier;
        /*double FRPower = (forward + horizontal + pivot);
        double FLPower = (forward - horizontal - pivot);
        double BRPower = (forward - horizontal + pivot);
        double BLPower = (forward + horizontal - pivot);*/
        //caiden.driveRawPower(FRPower, FLPower, BRPower, BLPower);
        //caiden.driveMotors(FRPower, FLPower, BRPower, BLPower);
        
        //linear slide
        if (raise){
            caiden.driveElevator(0.1);
        }
        else if (lower){
            caiden.driveElevator(-0.1);
        } 
        //Pinchy 
        if (open){
            caiden.openClaw();
        }
        else if  (close){
            caiden.closeClaw();
        }
        
        if(!(raise | lower)) {
            caiden.driveElevator(0);
        }
        //elbow code 
        if (raiseArm) {
            saveRaise = true;
        } else if (lowerArm) {
            saveRaise = false;
        }
        
        if(gamepad1.back) {
            if(headlightPower != 0) {
                headlightPower -= 0.01;
            }
        }
        
        if(gamepad1.start) {
            if(headlightPower != 1) {
                headlightPower += 0.01;
            }
        }
        
        //caiden.setHeadlightPower(headlightPower);
        
        /*if((left | right | straight) && !caiden.armAboveSafeLimit()) {
            caiden.armPosition(0.7);
        } else */if(saveRaise) {
            caiden.armPosition(0);
        } else {
            caiden.armPosition(1);
        }
        
     }

     @Override
    public void init() {
        caiden = new CaidenRobot(hardwareMap);
        anglePID = new PIDController(0.019, 0.01, 0.000);
     }
     
     private void sendTelemetry() {
         
        telemetry.addData("PID Enabled", enablePID);
        telemetry.addData("SetHeading", heading);
        telemetry.addData("Headlight Power", headlightPower);
        telemetry.addData("Heading Difference", headingDifference);
        telemetry.addData("Heading Difference Sign", headingDifferenceSign);
        
        caiden.updateTelemetry(telemetry);
        telemetry.update();
         
      }
    
    @Override
    public void loop() {
        moveDriveTrain();
        sendTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
