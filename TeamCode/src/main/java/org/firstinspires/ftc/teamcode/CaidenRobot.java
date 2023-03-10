package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.PwmControl;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class CaidenRobot {
    
    // The IMU sensor object
    BNO055IMU imu;   

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    
    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    private final DcMotorEx BRMotor;
    private final DcMotorEx BLMotor;
    private final DcMotorEx FRMotor;
    private final DcMotorEx FLMotor;
    
    private DcMotor headlightVoltage;
    
    private final PIDDriveMotor FRDrive;
    private final PIDDriveMotor FLDrive;
    private final PIDDriveMotor BRDrive;
    private final PIDDriveMotor BLDrive;
    
    private final double driveMotorP = 0.1;
    private final double driveMotorI = 0;
    private final double driveMotorD = 0;
    private final double driveMotorF = 0;
    private PIDFController FRDriveController = new PIDFController(driveMotorP, driveMotorI, driveMotorD, driveMotorF);
    private PIDFController FLDriveController = new PIDFController(driveMotorP, driveMotorI, driveMotorD, driveMotorF);
    private PIDFController BRDriveController = new PIDFController(driveMotorP, driveMotorI, driveMotorD, driveMotorF);
    private PIDFController BLDriveController = new PIDFController(driveMotorP, driveMotorI, driveMotorD, driveMotorF);

    private DcMotor LazySohum;
    private CRServo Jorj;
    private DistanceSensor distr;
    private ModernRoboticsI2cRangeSensor poleDistSensor;
    public AnalogInput Pot;
    public Servo Claw;
    public ColorSensor colorSensor;
    private ServoImplEx headlight;    
    
    private double armPosition = 0;
    
    private static PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(0, 5000, 5000);
    
    public DcMotorEx Slidey;
    private DcMotorEx Slidey2;
    PIDController elevatorController = new PIDController(.015, 0.0028, 0.0000);
    DistanceSensor distanceSensor;
    GyroSensor gyro;
    DistanceSensor revDistanceSensor;
    boolean stopElevator = true;
    
    private DcMotorEx HorizontalSlide;
    private static int ELEVATOR_HEIGHT = 900 / 5 * 12;

    int SAVED_ELEVATOR_POS = 0;
    int SAVED_LAZY_POS = 0;
    public final int rightLimit = -1450 / 60 * 20;
    public final int leftLimit = 440;
    double driveSpeedMultiplier = 1;
    
    private int shortHeight = 85 / 3 * 12;
    private int medHeight = 268 / 3 * 12;
    private int targetElevatorPosition = 0;
    private int targetTurretPosition = 0;

    private double lastElevatorPower = 0.0;
    private int lastElevatorPosition = 0;

    public CaidenRobot(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }
    
    public CaidenRobot(HardwareMap hardwareMap, boolean resetMotors) {
        
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

        FRDrive = new PIDDriveMotor(FRMotor, FRDriveController);
        FLDrive = new PIDDriveMotor(FLMotor, FLDriveController);
        BRDrive = new PIDDriveMotor(BRMotor, BRDriveController);
        BLDrive = new PIDDriveMotor(BLMotor, BLDriveController);

        Slidey = hardwareMap.get(DcMotorEx.class, "Slidey");
        Slidey2 = hardwareMap.get(DcMotorEx.class, "Slidey2");
        LazySohum = hardwareMap.get(DcMotor.class, "Lazy_Sohum");
        Claw = hardwareMap.get(Servo.class, "Claw");
        //Magnet = hardwareMap.get(TouchSensor.class, "magnet");
        Jorj = hardwareMap.get(CRServo.class, "Jorj");
        Pot = hardwareMap.get(AnalogInput.class, "Pot");
        HorizontalSlide = hardwareMap.get(DcMotorEx.class, "horiz_Slide");
        distr = hardwareMap.get(DistanceSensor.class, "distr");
        //poleDistSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "MRRange");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        headlight = hardwareMap.get(ServoImplEx.class, "headlight");
        headlight.setPwmRange(pwmRange);

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //HorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        
        
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

        
        /*Thread elevatorThread = new Thread(() -> {
            DcMotorEx Slidey = hardwareMap.get(DcMotorEx.class, "Slidey");
            DcMotorEx Slidey2 = hardwareMap.get(DcMotorEx.class, "Slidey2");
            Slidey.setDirection(DcMotorSimple.Direction.REVERSE);
            Slidey2.setDirection(DcMotorSimple.Direction.REVERSE);

            if(resetMotors) {
                Slidey.setMode(RunMode.STOP_AND_RESET_ENCODER);
                Slidey2.setMode(RunMode.STOP_AND_RESET_ENCODER);
            }

            Slidey.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Slidey2.setMode(RunMode.RUN_WITHOUT_ENCODER);

            while(stopRobot.getAsBoolean()) {
                double power = elevatorPowerAtomic.get();
                Slidey.setPower(power);
                Slidey2.setPower(power);
                elevatorPositionAtomic.set(Slidey.getCurrentPosition());
            }

            Slidey.setPower(0);
            Slidey2.setPower(0);

        });

        elevatorThread.start();*/
        /*
         *  Other Sensors
         */
         
        //magnet = hardwareMap.get(TouchSensor.class, "magnet");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        //revDistanceSensor = hardwareMap.get(DistanceSensor.class, "revDistance");
        
    }

    public void stop() {
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        Slidey.setPower(0);
        Slidey2.setPower(0);
        LazySohum.setPower(0);
        Jorj.setPower(0);
        HorizontalSlide.setPower(0);
    }
    public int turretDisplacement(){
        return Math.abs(LazySohum.getCurrentPosition() - LazySohum.getTargetPosition());
    }
    public void setHeadlightPower(double power) {
         headlight.setPosition(power);
    }
    
    public void enableHeadlight() {
        headlightVoltage.setPower(1);
    }
    
    public void disableHeadlight() {
        headlightVoltage.setPower(0);
    }

    public double getDistance() {
        return distr.getDistance(DistanceUnit.CM);
    }
    
    public void openClaw() {
        Claw.setPosition(0.84);
    }
    
    public void closeClaw() {
        Claw.setPosition(0.50);
    }
    
    public void driveElevator(double power) {

        int elevatorPosition = Slidey.getCurrentPosition();


        if(targetElevatorPosition < 10 && elevatorPosition < 10 && power == 0) {
            lastElevatorPower = (0.0);
        } else if(power == 0) {
            if (!stopElevator){
                stopElevator = true;
                targetElevatorPosition = elevatorPosition;
            }            
            double elevatorPower = Range.clip(elevatorController.calculate(elevatorPosition, targetElevatorPosition), -.7, 1);
            lastElevatorPower = elevatorPower;
        } else if((power < 0) && (elevatorPosition > 8)) {
            stopElevator = false;
            double elevatorPower = Range.clip(elevatorController.calculate(elevatorPosition, 20), -0.7, 0);
            lastElevatorPower = elevatorPower;
        } else if (power > 0) {
            stopElevator = false;
            double elevatorPower = Range.clip(elevatorController.calculate(elevatorPosition, ELEVATOR_HEIGHT), -0.3, .7);
            lastElevatorPower = elevatorPower;
        }

        Slidey.setPower(lastElevatorPower);
        Slidey2.setPower(lastElevatorPower);

    }

    public void horizontalSlideOut() {
        HorizontalSlide.setPower(1);
        HorizontalSlide.setTargetPosition(290);
        HorizontalSlide.setMode(RunMode.RUN_TO_POSITION);

    }
    public void horizontalSlideIn() {
        HorizontalSlide.setPower(-1);
        HorizontalSlide.setTargetPosition(0);
        HorizontalSlide.setMode(RunMode.RUN_TO_POSITION);

    }
    public void goToElevatorPosition(int position) {
        targetElevatorPosition = Range.clip(position, 0, ELEVATOR_HEIGHT);
        driveElevator(0);

        //elevatorPowerAtomic.set(Range.clip(elevatorController.calculate(getElevatorPosition(), targetElevatorPosition), -0.6, 0.7));


        //Slidey.setPower(elevatorPowerAtomic.get());
        //Slidey2.setPower(elevatorPowerAtomic.get());
        //Slidey.setPower(elevatorPower);
        //Slidey2.setPower(elevatorPower);
    }
    
    // Is it safe to move the turret?
    public static final int SAFE_ELEVATOR_POSITION = 100;
    public boolean safeToMoveTurret() {
        return Slidey.getCurrentPosition() > SAFE_ELEVATOR_POSITION;
    }

    public void power(double power) {
        Slidey.setPower(power);
        Slidey2.setPower(power);
    }
    
    public void stopArm() {
        
    }

    public void poorPID() {
        power(.4);
        Slidey.setMode(RunMode.RUN_TO_POSITION);
        Slidey.setTargetPosition(500);
        Slidey2.setPower(Slidey.getPower());
        //if (Math.abs(Slidey.getCurrentPosition() - Slidey.getTargetPosition()) < 10 )
    }
    public void lazyL() {
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(leftLimit);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.4);
        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 60;
        } else {
            LazySohum.setPower(0);
        }
        if (300 < LazySohum.getCurrentPosition() && LazySohum.getPower() == 0){
            LazySohum.setTargetPosition(leftLimit);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.4);
            driveElevator(-1);
        }

    }
    public void lazyR () {
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(rightLimit);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.4);
        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 60;
        } else {
            LazySohum.setPower(0);
        }
        if (LazySohum.getCurrentPosition() < -300 && LazySohum.getPower() == 0){
            LazySohum.setTargetPosition(rightLimit);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.4);
            driveElevator(-1);
        }
    }
    public void lazyS(){
        if(safeToMoveTurret()) {
            LazySohum.setTargetPosition(0);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.4);
        } else if(targetElevatorPosition < SAFE_ELEVATOR_POSITION) {
            LazySohum.setPower(0);
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 60;
        } else {
            LazySohum.setPower(0);
        }
        if (-100 < LazySohum.getCurrentPosition() && LazySohum.getCurrentPosition() < 100 && LazySohum.getPower() == 0){
            LazySohum.setTargetPosition(0);
            LazySohum.setMode(RunMode.RUN_TO_POSITION);
            LazySohum.setPower(0.4);
            driveElevator(-1);
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
            targetElevatorPosition = SAFE_ELEVATOR_POSITION + 200;
        } else {
            LazySohum.setPower(0);
        }
       
    }
    
    public void updateElevatorTargetPosition(int position){
        position = Range.clip(position, 6, ELEVATOR_HEIGHT);
        targetElevatorPosition = position;
    }
    public void updateTurretTargetPosition(int position){
        position = Range.clip(position, rightLimit, leftLimit);
        targetTurretPosition = position;
        if(turretDisplacement() > 3){

        }
    }
    public boolean elevatorIsInPosition() {
        double elevatorPosition = Slidey.getCurrentPosition();
        final int ELEVATOR_TOLERANCE = 90;
        return elevatorPosition > (targetElevatorPosition - ELEVATOR_TOLERANCE) &&
                elevatorPosition < (targetElevatorPosition + ELEVATOR_TOLERANCE);
    }
    
    public void driveRawPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        if(Slidey.getCurrentPosition() >= 1700) {
            driveSpeedMultiplier = 0.5;
        } else {
            driveSpeedMultiplier = 1;
        }
        FRMotor.setPower(frontRightPower * driveSpeedMultiplier);
        FLMotor.setPower(frontLeftPower * driveSpeedMultiplier);
        BRMotor.setPower(backRightPower * driveSpeedMultiplier);
        BLMotor.setPower(backLeftPower * driveSpeedMultiplier);
    }
    
    public void driveMotors(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        if(Slidey.getCurrentPosition() >= 700) {
            driveSpeedMultiplier = 0.5;
        } else {
            driveSpeedMultiplier = 1;
        }
        FRDrive.setVelocity(4000 * frontRightPower * driveSpeedMultiplier);
        FLDrive.setVelocity(4000 * frontLeftPower * driveSpeedMultiplier);
        BRDrive.setVelocity(4000 * backRightPower * driveSpeedMultiplier);
        BLDrive.setVelocity(4000 * backLeftPower * driveSpeedMultiplier);
        
        FRDrive.update();
        FLDrive.update();
        BRDrive.update();
        BLDrive.update();
        
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
    
    public void goToPivotPosition(int position) {
        goToPosition(position, -position, position, -position);
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
    
    public void moveArm(double power) {
        if(power > 0 && Pot.getVoltage() > MAX_ARM_VOLTAGE) {
            armMoving = true;
            Jorj.setPower(power);
        } else if (power < 0 && Pot.getVoltage() < MIN_ARM_VOLTAGE) {
            armMoving = true;
            Jorj.setPower(power);
        } else {
            Jorj.setPower(0);
        }
        
    }
    
    public void armPosition(double position) {
        armPosition = MIN_ARM_VOLTAGE - ((MIN_ARM_VOLTAGE - MAX_ARM_VOLTAGE) * position);
        Jorj.setPower(-armController.calculate(Pot.getVoltage(), armPosition));
    }
    
    public void armRawPosition(double position) {
        Jorj.setPower(-armController.calculate(Pot.getVoltage(), position));
    }
    
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
        //telemetry.addData("MR Distance", poleDistSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("BRcount", BRMotor.getCurrentPosition());
        //telemetry.addData("FRcount", FRMotor.getCurrentPosition());
        telemetry.addData("FRcount", FRMotor.getCurrentPosition());
        telemetry.addData("Elevator Position", Slidey.getCurrentPosition());
        telemetry.addData("Elevator power", lastElevatorPower);
        //telemetry.addData("Turret", LazySohum.getCurrentPosition());
        //telemetry.addData("Magnet", Magnet.getValue());
        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        //telemetry.addData("Servo Pow", Jorj.getPower());
        //telemetry.addData("Where is the Pot", Pot.getVoltage());
        //telemetry.addData("Distance to r", distr.getDistance(DistanceUnit.CM));
        //telemetry.addData("arm desired pos", armPosition);
        //telemetry.addData("Red", colorSensor.red());
        //telemetry.addData("Blue", colorSensor.blue());
        //telemetry.addData("Green", colorSensor.green());
        telemetry.addData("horiz slide", HorizontalSlide.getCurrentPosition());
    }
    
    public int getFRMotorCount() {
       return FRMotor.getCurrentPosition(); 
    }
}