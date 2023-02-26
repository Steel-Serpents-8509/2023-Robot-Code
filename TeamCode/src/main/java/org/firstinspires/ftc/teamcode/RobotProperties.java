package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.sun.tools.javac.tree.Pretty;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;

public class RobotProperties {

    private RobotProperties() { throw new IllegalStateException("Utility Class"); }
    @SuppressLint("SdCardPath")
    private static final File file = new File("/sdcard/FIRST/robot.properties");
    private static final Properties properties = new Properties();
    private static Telemetry telemetry;

    public static final String VALUE_PREFIX = "value.";
    public static final String PID_PREFIX = "pid.";

    static {
        try {
            file.createNewFile();
            if(file.isFile()) {
                reloadData();
            } else {
                throw new IllegalStateException("Properties file is not a file");
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static String getProperty(String propertyName) {
        return (String) properties.get(propertyName);
    }


    private static void updateFile() {
        try {
            properties.store(new FileWriter(file), null);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static void reloadData() {
        try {
            properties.load(new FileInputStream(file));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    private static double getDouble(String name, double defaultValue) {
        try {
            reloadData();
            return Double.parseDouble((String)properties.get(VALUE_PREFIX + name));
        } catch (NumberFormatException | NullPointerException e) {
            properties.setProperty(VALUE_PREFIX + name, String.valueOf(defaultValue));
            updateFile();
            return defaultValue;
        }
    }

    public static int getIntValue(String intName, int defaultValue) {
        int ret;
        try {
            ret = Integer.parseInt((String)properties.get(VALUE_PREFIX + intName));
        } catch (NumberFormatException | NullPointerException e) {
            properties.setProperty(VALUE_PREFIX + intName, String.valueOf(defaultValue));
            updateFile();
            ret = defaultValue;
        }
        return ret;
    }

    public static double getDoubleValue(String doubleName, double defaultValue) {
        return getDouble(VALUE_PREFIX + doubleName, defaultValue);
    }

    public static PIDCoefficients getPIDCoefficients(@NonNull String pidName, @NonNull PIDCoefficients defaultValues) {
        return new PIDCoefficients(getDouble(PID_PREFIX + pidName + ".p", defaultValues.p),
                getDouble(PID_PREFIX + pidName + ".i", defaultValues.i),
                getDouble(PID_PREFIX + pidName +  ".d", defaultValues.d)
        );
    }

}
