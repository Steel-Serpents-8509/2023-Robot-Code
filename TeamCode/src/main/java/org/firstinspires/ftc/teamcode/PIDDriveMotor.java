package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

public class PIDDriveMotor {

    DcMotorEx motor;
    PIDFController controller;
    
    double targetVelocity = 0;
    double workingTargetVelocity = 0;
    double maxForwardAcceleration = 6;
    double maxReverseAcceleration = 6;
    double lastTimestamp = 0;
    double lastPower = 0;
    boolean coastInReverse = true;
    long previousTimestamp = 0;
    
    double previousVelocity = 0;
    
    double maxPower = 0.8;
    
    public PIDDriveMotor(DcMotorEx motor, PIDFController pid) {
        this.motor = motor;
        controller = pid;
    }
    
    public void setVelocity(double v) {
        targetVelocity = v;
    }
    
    public void setMaxForwardAcceleration(double a) {
        maxForwardAcceleration = a;
    }
    
    public void update() {
        long currentTimestamp = System.nanoTime();
        double currentVelocity = motor.getVelocity();
        if(previousTimestamp == 0) {
            previousTimestamp = currentTimestamp;
        }
        
        int period = (int)(currentTimestamp - previousTimestamp);
        
        /*if(currentVelocity > 0 && currentVelocity > targetVelocity && workingTargetVelocity > currentVelocity) {
            workingTargetVelocity = currentVelocity;
        } else if (currentVelocity < 0 && currentVelocity < targetVelocity && workingTargetVelocity < currentVelocity) {
            workingTargetVelocity = currentVelocity;
        } else 
        
        if(Math.abs(targetVelocity) < 100) {
            workingTargetVelocity = currentVelocity;
        } else */
        
        if(workingTargetVelocity < targetVelocity) {
            //                                                  nano seconds to ms
            double workingAcceleration = (maxForwardAcceleration * (period / 1000000));
            if((targetVelocity - workingTargetVelocity) <= workingAcceleration) {
                workingTargetVelocity = targetVelocity;
            } else {
                workingTargetVelocity += workingAcceleration;
            }
        } else if(workingTargetVelocity > targetVelocity) {
            //                                                  nano seconds to ms
            double workingAcceleration = (maxReverseAcceleration * (period / 1000000));
            if(((workingTargetVelocity - targetVelocity) <= workingAcceleration)) {
                workingTargetVelocity = targetVelocity;
            } else {
                workingTargetVelocity -= workingAcceleration;
            }
        }

        //lastPower = Range.clip(maxPower * controller.calculate(currentVelocity, workingTargetVelocity), (targetVelocity >= 0 ? 0 : -maxPower), (targetVelocity <= 0 ? 0 : maxPower));
        lastPower = Range.clip(maxPower * controller.calculate(currentVelocity, workingTargetVelocity), -maxPower, maxPower);
        motor.setPower(lastPower);
        previousVelocity = currentVelocity;
        previousTimestamp = currentTimestamp;
    }

    public double getLastPower() {
        return lastPower;
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }
    
    public double getWorkingTargetVelocity() {
        return workingTargetVelocity;
    }
    
}
