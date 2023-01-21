package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.Collections;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class CustomVision {

    private final String tfod_model;
    private final boolean useDefault;
    private final double ZOOM = 1;

    /*private static final String[] LABELS = {
            "nut",
            "snake",
            "sohum"
    };*/
    
    private static final String[] LABELS = {
            "circle",
            "square",
            "triangle"
    };
    
    // ORDER IS IMPORTANT DO NOT CHANGE
     private static final String[] DEFAULT_LABELS = {
        "snake",
        "nut",
        "sohum"
    };

    private static final String VUFORIA_KEY = "AXCmyyr/////AAABmaAlOjhYSUMuqnC79L+csYMS30s8nhVrbQ5s0YED5j3dyq2vYu7aHhvrFpFMJaG6L2hUnXm98+QdvRoeRcALbC0iCcXdY+VcdSIFPHwve6Qr6Oyjz0tsfcCn2fGkTyP5gOO6Ot/+WglV+21XFIeC2Y0zcj+9/awni80ki4xZJID/5dPXeqE9T2VeHdrwnzVyq8DoOzB1LoNL0k1k9kkETyffbDOjQz2qnXgyoMWRn1loy0tZzpjC85OMpoV3bDlF/TJ9CqM8KyVgN/t3/gn6kXjmMxFvU5wWRCSxB23gKeFEIjVrlOJfHyShmrMMuYIbTjkzPY3nV9ctXZ1tj9O6BG6Q9QgEV7TXJvysjkgq5Yl0";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    
    private List<Recognition> cachedRecognitions = Collections.emptyList();
    
    public CustomVision(HardwareMap hardwareMap) {
        this.useDefault = true;
        this.tfod_model = "PowerPlay.tflite";
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(ZOOM, 16.0/9.0);
        }
    }
    
    public CustomVision(HardwareMap hardwareMap, String tfod_model) {
        this.useDefault = false;
        this.tfod_model = tfod_model;
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(ZOOM, 16.0/9.0);
        }
    }

        public List<Recognition> lookForObject() {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    cachedRecognitions = updatedRecognitions;

                    
                    return updatedRecognitions;
                }
                return cachedRecognitions;
           }
           //telemetry.addData;
           return Collections.emptyList();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfodParameters.isModelQuantized = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        if(useDefault) {
            tfod.loadModelFromAsset(tfod_model, DEFAULT_LABELS);
        } else {
            tfod.loadModelFromFile(tfod_model, LABELS);
        }
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    
    public void updateTelemetry(Telemetry tel) {
        tel.addData("TFLite Model", tfod_model);
    }
}
 