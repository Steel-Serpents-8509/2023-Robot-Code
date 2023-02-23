package org.firstinspires.ftc.teamcode.testing;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.CaidenRobot;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.ButtonDebounce;
@TeleOp

public class TestTeleop extends OpMode {

    CaidenRobot caiden;
    PIDController anglePID;
    double forward = 0;
    double horizontal = 0;
    double pivot = 0;
    int heading = 0;
    double headlightPower = 0;
    int shortHeight = 915 / 12 * 5;
    int medHeight = 1575 / 12 * 5;
    int topHeight = 2050 / 12 * 5;

    boolean strafe1;
    boolean strafe2;
    boolean raise;
    boolean lower;


    boolean close;
    boolean open;
    boolean left;
    boolean right;
    boolean straight;
    boolean saveRaise = true;
    boolean first;
    boolean second;
    boolean third;
    boolean fourth;
    boolean Inc;
    boolean Outc;
    boolean test;

    boolean In;
    boolean Out;
    double headingDifference;
    double headingDifferenceSign;
    double headingAdjustment = 0;

    boolean enablePID = true;

    ButtonDebounce rotateCW90 = new ButtonDebounce();
    ButtonDebounce rotateCCW90 = new ButtonDebounce();
    ButtonDebounce compassAdjustRight = new ButtonDebounce();
    ButtonDebounce compassAdjustLeft = new ButtonDebounce();
    ElapsedTime loopTime = new ElapsedTime();

    public void moveDriveTrain() {

        boolean currentY = gamepad1.y;
        boolean currentX = gamepad1.x;

        forward = -(gamepad1.left_stick_y * 0.7);
        horizontal = -(gamepad1.left_stick_x * 0.7);
        //pivot = -(gamepad1.left_stick_x * 0.6);

        raise = gamepad2.right_trigger > .1;
        lower = gamepad2.left_trigger > .1;

        Outc = gamepad2.left_stick_y > 0.5;
        Inc = gamepad2.left_stick_y < -0.5;

        first = gamepad2.dpad_down;
        second = gamepad2.dpad_left;
        third = gamepad2.dpad_up;
        fourth = gamepad2.dpad_right;

        //quick = gamepad2.a;

        close = gamepad1.right_trigger > .1;
        open = gamepad1.left_trigger > .1;

        In = gamepad1.a;
        Out = gamepad1.b;
        test = gamepad1.x;

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

        if(compassAdjustRight.getButton(gamepad1.dpad_right)) {
            headingAdjustment += 3;
        } else if(compassAdjustLeft.getButton(gamepad1.dpad_left)) {
            headingAdjustment -= 3;
        }

        if (left){
            caiden.lazyL();
            caiden.horizontalSlideOut();
        } else if (right){
            caiden.lazyR();
            caiden.horizontalSlideOut();
        } else if (straight){
            caiden.lazyS();
            caiden.horizontalSlideIn();
            //caiden.updateElevatorTargetPosition(5);
        }
        //robot stays straight
        //if(currentY) {
            //enablePID = false;
        //}
        if(currentX) {
            enablePID = false;
        }

        if(enablePID) {
            headingDifference = (caiden.getCachedHeading() - heading) % 360;

            if (Math.abs(headingDifference) > 180) {
                headingDifferenceSign = 360 - Math.abs(headingDifference);
                if (headingDifference > 0) {
                    headingDifference = -headingDifferenceSign;
                } else {
                    headingDifference = headingDifferenceSign;
                }
            }
            pivot = anglePID.calculate(headingDifference + headingAdjustment, 0);
        } else {
            pivot = 0;
        }

        if(Out){
            caiden.horizontalSlideOut();
        }
        if(In){
            caiden.horizontalSlideIn();
        }
        //Make robot go vroom vroom
        if (strafe1){
            double power = 0.7;
            caiden.driveRawPower(power, -power, -power, power);
        }
        else if (strafe2){
            double power = 0.7;
            caiden.driveRawPower(-power, power, power, -power);
        } else {
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
            caiden.driveRawPower(FRPower, FLPower, BRPower, BLPower);
            //caiden.driveMotors(FRPower, FLPower, BRPower, BLPower);

        }
        if (first){
            caiden.updateElevatorTargetPosition(0);
            //saveRaise = true;
        }
        if (second){
            caiden.updateElevatorTargetPosition(shortHeight);
        }
        if (third){
            caiden.updateElevatorTargetPosition(medHeight);
        }
        if (fourth){
            caiden.updateElevatorTargetPosition(topHeight);
            //saveRaise = false;
        }
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
        if (test){
            caiden.resets();
        }
        if(!(raise || lower)) {
            caiden.driveElevator(0);
        }
        //elbow code
        if(Outc){
            caiden.horizontalSlideOut();
        }
        if(Inc){
            caiden.horizontalSlideIn();
        }

    }

    @Override
    public void init() {
        caiden = new CaidenRobot(hardwareMap, true);
        anglePID = new PIDController(0.019, 0.01, 0.000);
        PhotonCore.enable();
    }

    private void sendTelemetry() {

        telemetry.addData("PID Enabled", enablePID);
        telemetry.addData("SetHeading", heading);
        telemetry.addData("Headlight Power", headlightPower);
        telemetry.addData("Heading Difference", headingDifference);
        telemetry.addData("Heading Difference Sign", headingDifferenceSign);
        telemetry.addData("Heading Adjustment", headingAdjustment);

        telemetry.addData("Error in turret", caiden.turretDisplacement());

        caiden.updateTelemetry(telemetry);
        telemetry.addData("Loop Time", loopTime.milliseconds());
        telemetry.update();
        loopTime.reset();

    }

    @Override
    public void loop() {
        moveDriveTrain();
        sendTelemetry();
        caiden.updateTelemetry(telemetry);
    }
}