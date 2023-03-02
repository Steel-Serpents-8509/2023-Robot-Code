package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.PwmControl;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;

public class CaidenRobot {
    
    // The IMU sensor object
    BNO055IMU imu;   

    // State used for updating telemetry
    Orientation angles;

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    private final DcMotorEx BRMotor;
    private final DcMotorEx BLMotor;
    private final DcMotorEx FRMotor;
    private final DcMotorEx FLMotor;

    private DcMotor headlightVoltage;
    private DcMotor LazySohum;

    private DistanceSensor distr;
    private ModernRoboticsI2cRangeSensor poleDistSensor;

    public Servo Claw;
    public ColorSensor colorSensor;
    private ServoImplEx headlight;    
    
    private double armPosition = 0;
    
    private static PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(0, 5000, 5000);
    
    public DcMotorEx Slidey;
    private DcMotorEx Slidey2;

    private final PIDFCoefficients elevatorPIDFCoefficients;
    private final PIDFController elevatorController;

    DistanceSensor distanceSensor;
    DistanceSensor revDistanceSensor;
    boolean stopElevator = true;
    
    private DcMotorEx HorizontalSlide;
    private static int ELEVATOR_HEIGHT = 1060;

    int SAVED_ELEVATOR_POS = 0;
    int SAVED_LAZY_POS = 0;
    public final int rightLimit = -449;
    public final int leftLimit = 450;
    double driveSpeedMultiplier = 1;

    private static final int SHORT_ELEVATOR_HEIGHT = 465;
    private static final int MEDIUM_ELEVATOR_HEIGHT = 777;
    private static final int HIGH_ELEVATOR_HEIGHT = 1060;

    private int targetElevatorPosition = 0;
    private int targetTurretPosition = 0;

    private double lastElevatorPower = 0.0;
    private int lastElevatorPosition = 0;

    public CaidenRobot(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }
    
    public CaidenRobot(HardwareMap hardwareMap, boolean resetMotors) {
        PhotonCore.enable();
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        /*
         *  Setup Motors
         */
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        FRMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");

        Slidey = hardwareMap.get(DcMotorEx.class, "Slidey");
        Slidey2 = hardwareMap.get(DcMotorEx.class, "Slidey2");
        LazySohum = hardwareMap.get(DcMotor.class, "Lazy_Sohum");
        Claw = hardwareMap.get(Servo.class, "Claw");

        HorizontalSlide = hardwareMap.get(DcMotorEx.class, "horiz_Slide");
        distr = hardwareMap.get(Rev2mDistanceSensorEx.class, "distr");
        colorSensor = hardwareMap.get(RevColorSensorV3Ex.class, "colorSensor");
        headlight = hardwareMap.get(ServoImplEx.class, "headlight");
        headlight.setPwmRange(pwmRange);
        getDistance();
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if(resetMotors) {
            BRMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            BLMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            FRMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            FLMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            Slidey.setMode(RunMode.STOP_AND_RESET_ENCODER);
            Slidey2.setMode(RunMode.STOP_AND_RESET_ENCODER);
            LazySohum.setMode(RunMode.STOP_AND_RESET_ENCODER);
            HorizontalSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);
        }
        
        BRMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        LazySohum.setMode(RunMode.RUN_USING_ENCODER);
        
        Slidey.setMode(RunMode.RUN_WITHOUT_ENCODER);
        Slidey2.setMode(RunMode.RUN_WITHOUT_ENCODER);

        HorizontalSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
        elevatorPIDFCoefficients = RobotProperties.getPIDCoefficients("elevator", new PIDFCoefficients(0.02, 0.0, 0.0, 0.3));
        elevatorController = new PIDFController(
                elevatorPIDFCoefficients.p,
                elevatorPIDFCoefficients.i,
                elevatorPIDFCoefficients.d,
                elevatorPIDFCoefficients.f
        );
    }

    public void stop() {
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        Slidey.setPower(0);
        Slidey2.setPower(0);
        LazySohum.setPower(0);

        HorizontalSlide.setPower(0);
    }
    public int turretDisplacement(){
        return Math.abs(LazySohum.getCurrentPosition() - LazySohum.getTargetPosition());
    }
    public void setHeadlightPower(double power) {
         headlight.setPosition(power);
    }
    


    public double getDistance() {
        return distr.getDistance(DistanceUnit.CM);
    }

    final double OPEN_CLAW_VALUE = RobotProperties.getDoubleValue("open_claw_value", 0.8);
    final double CLOSE_CLAW_VALUE = RobotProperties.getDoubleValue("close_claw_value" , 0.0);
    public void openClaw() {
        Claw.setPosition(OPEN_CLAW_VALUE);
    }
    
    public void closeClaw() {
        Claw.setPosition(CLOSE_CLAW_VALUE);
    }
    
    public void driveElevator(double power) {

        lastElevatorPosition = Slidey.getCurrentPosition();

        if(targetElevatorPosition < 20 && lastElevatorPosition < 20 && power == 0) {
            lastElevatorPower = (0.0);
        } else if(power == 0) {
            if (!stopElevator){
                stopElevator = true;
                targetElevatorPosition = lastElevatorPosition;
            }
            lastElevatorPower = Range.clip(elevatorController.calculate(lastElevatorPosition, targetElevatorPosition), -.7, 1);
        } else if((power < 0) && (lastElevatorPosition > 8)) {
            stopElevator = false;
            lastElevatorPower = Range.clip(elevatorController.calculate(lastElevatorPosition, 10), -0.7, 0);
        } else if (power > 0) {
            stopElevator = false;
            lastElevatorPower = Range.clip(elevatorController.calculate(lastElevatorPosition, ELEVATOR_HEIGHT), -0.3, .7);
        }

        Slidey.setPower(lastElevatorPower);
        Slidey2.setPower(lastElevatorPower);

    }


    public void horizontalSlideOut() {
        HorizontalSlide.setPower(1);
        HorizontalSlide.setTargetPosition(290);
        HorizontalSlide.setMode(RunMode.RUN_TO_POSITION);

    }

    public void horizontalSlideOutKinda() {
        HorizontalSlide.setPower(1);
        HorizontalSlide.setTargetPosition(210);
        HorizontalSlide.setMode(RunMode.RUN_TO_POSITION);

    }

    public void horizontalSlideIn() {
        HorizontalSlide.setPower(-1);
        HorizontalSlide.setTargetPosition(0);
        HorizontalSlide.setMode(RunMode.RUN_TO_POSITION);

    }
    public void goToElevatorPosition(int position) {
        //TODO: refactor. It is not clear that driveElevator(0) tried to go to the target position
        targetElevatorPosition = Range.clip(position, 0, ELEVATOR_HEIGHT);
        driveElevator(0);
    }
    
    // Is it safe to move the turret?
    public static final int SAFE_ELEVATOR_POSITION = 150;
    public boolean safeToMoveTurret() {
        return Slidey.getCurrentPosition() > SAFE_ELEVATOR_POSITION;
    }

    public void power(double power) {
        Slidey.setPower(power);
        Slidey2.setPower(power);
    }
    public int getElevatorHeight(){
        return lastElevatorPosition;
    }
    public void poorPID() {
        power(.4);
        Slidey.setMode(RunMode.RUN_TO_POSITION);
        Slidey.setTargetPosition(500);
        Slidey2.setPower(Slidey.getPower());

    }

    public void lazyL() {
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(leftLimit);
            LazySohum.setPower(0.5);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);

        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION && (Math.abs(LazySohum.getCurrentPosition() - leftLimit) > 150)) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 100;
        }
    }
    public void lazyR () {
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(rightLimit);
            LazySohum.setPower(0.5);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);

        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION && (Math.abs(LazySohum.getCurrentPosition() - rightLimit) > 150)) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 100;
        }
    }
    public void lazyS(){
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(0);
            LazySohum.setPower(0.5);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);

        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION && (Math.abs(LazySohum.getCurrentPosition()) > 150)) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 100;
        }
    }
    
    public void lazyGoToPosition(int position) {
        position = Range.clip(position, leftLimit, rightLimit);
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(position);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.7);
        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 100;
        } else {
            LazySohum.setPower(0);
        }
       
    }

    public void goToLowElevatorPosition() {
        updateElevatorTargetPosition(SHORT_ELEVATOR_HEIGHT);
        driveElevator(0);
    }

    public void goToMediumElevatorPosition() {
        updateElevatorTargetPosition(MEDIUM_ELEVATOR_HEIGHT);
        driveElevator(0);

    }

    public void goToHighElevatorPosition() {
        updateElevatorTargetPosition(HIGH_ELEVATOR_HEIGHT);
        driveElevator(0);

    }

    public void updateElevatorTargetPosition(int position){
        position = Range.clip(position, 0, ELEVATOR_HEIGHT);
        targetElevatorPosition = position;
    }

    public boolean elevatorIsInPosition() {
        double elevatorPosition = Slidey.getCurrentPosition();
        final int ELEVATOR_TOLERANCE = 30;
        return elevatorPosition > (targetElevatorPosition - ELEVATOR_TOLERANCE) &&
                elevatorPosition < (targetElevatorPosition + ELEVATOR_TOLERANCE);
    }
    
    public void driveRawPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        if(Slidey.getCurrentPosition() >= 900) {
            driveSpeedMultiplier = 0.5;
        } else {
            driveSpeedMultiplier = 1;
        }
        FRMotor.setPower(frontRightPower * driveSpeedMultiplier);
        FLMotor.setPower(frontLeftPower * driveSpeedMultiplier);
        BRMotor.setPower(backRightPower * driveSpeedMultiplier);
        BLMotor.setPower(backLeftPower * driveSpeedMultiplier);
    }

    public void driveRawPowerInAuto(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {

        FRMotor.setPower(frontRightPower * driveSpeedMultiplier);
        FLMotor.setPower(frontLeftPower * driveSpeedMultiplier);
        BRMotor.setPower(backRightPower * driveSpeedMultiplier);
        BLMotor.setPower(backLeftPower * driveSpeedMultiplier);
    }
    
    public void goToForwardPosition(int position) {
        goToPosition(position, position, position, position);
    }
    
    public void goToStrafePosition(int position) {
        goToPosition(position, -position, -position, position);
    }

    public void goToPosition(int FRPosition, int FLPosition, int BRPosition, int BLPosition) {
       
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        
        FLMotor.setTargetPosition(FLPosition / 2);
        FRMotor.setTargetPosition(FRPosition / 2);
        BLMotor.setTargetPosition(BLPosition / 2);
        BRMotor.setTargetPosition(BRPosition / 2);
        
        BRMotor.setMode(RunMode.RUN_TO_POSITION);
        BLMotor.setMode(RunMode.RUN_TO_POSITION);
        FRMotor.setMode(RunMode.RUN_TO_POSITION);
        FLMotor.setMode(RunMode.RUN_TO_POSITION);
        
        
        FRMotor.setPower(0.7);
        FLMotor.setPower(0.7);
        BRMotor.setPower(0.7);
        BLMotor.setPower(0.7);
        
    }
    
    public boolean isBusy() {
        return FRMotor.isBusy() || FLMotor.isBusy() || BRMotor.isBusy() || BLMotor.isBusy();
    }
    
    public void reset() {
        BRMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //Slidey.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void resetDrivetrain() {
        BRMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //Slidey.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    }
    
    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void resets() {
        HorizontalSlide.setPower(-.7);
    }
    public double getCachedHeading() {
        if(angles == null) {
            return getRawHeading();
        }
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
        
    }
    
    PIDFController armController = new PIDFController(0.5, 0.02, 0, 0);
    
    static final double MAX_ARM_VOLTAGE = .5;
    static final double SAFE_VOLTAGE = 2.1;
    static final double MIN_ARM_VOLTAGE = 2.14;
    double savedArmPosition = MIN_ARM_VOLTAGE;
    boolean armMoving = false;
    boolean armPIDEnabled = true;
    

    

    
    public double colorSensorRed() {
        return colorSensor.red();
    }
    
    public double colorSensorGreen() {
        return colorSensor.green();
    }
    
    public double colorSensorBlue() {
        return colorSensor.blue();
    }
    
    public void updateTelemetry(Telemetry telemetry) {
//        //telemetry.addData("MR Distance", poleDistSensor.getDistance(DistanceUnit.CM));
//        telemetry.addData("BRcount", BRMotor.getCurrentPosition());
//        //telemetry.addData("FRcount", FRMotor.getCurrentPosition());
//        telemetry.addData("FRcount", FRMotor.getCurrentPosition());
//        telemetry.addData("Elevator power", lastElevatorPower);
//        telemetry.addData("Last Elevator Position", lastElevatorPosition);
//        //telemetry.addData("Turret", LazySohum.getCurrentPosition());
//        //telemetry.addData("Magnet", Magnet.getValue());
//        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        getRawHeading();
        telemetry.addData("Distance to r", distr.getDistance(DistanceUnit.CM));
//        //telemetry.addData("arm desired pos", armPosition);
//        //telemetry.addData("Red", colorSensor.red());
//        //telemetry.addData("Blue", colorSensor.blue());
//        //telemetry.addData("Green", colorSensor.green());
//        telemetry.addData("horiz slide", HorizontalSlide.getCurrentPosition());
    }
    
    public int getFRMotorCount() {
       return FRMotor.getCurrentPosition(); 
    }
}