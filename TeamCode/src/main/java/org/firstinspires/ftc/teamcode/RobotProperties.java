package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    private static int getInt(String name, int defaultValue) {
        try {
            reloadData();
            return Integer.parseInt((String)properties.get(VALUE_PREFIX + name));
        } catch (NumberFormatException | NullPointerException e) {
            properties.setProperty(VALUE_PREFIX + name, String.valueOf(defaultValue));
            updateFile();
            return defaultValue;
        }
    }

    public static int getIntValue(String intName, int defaultValue) {
        return getInt(VALUE_PREFIX + intName, defaultValue);
    }

    public static double getDoubleValue(String doubleName, double defaultValue) {
        return getDouble(VALUE_PREFIX + doubleName, defaultValue);
    }

    public static PIDFCoefficients getPIDCoefficients(@NonNull String pidName, @NonNull PIDFCoefficients defaultValues) {
        return new PIDFCoefficients(getDouble(PID_PREFIX + pidName + ".p", defaultValues.p),
                getDouble(PID_PREFIX + pidName + ".i", defaultValues.i),
                getDouble(PID_PREFIX + pidName +  ".d", defaultValues.d),
                getDouble(PID_PREFIX + pidName + ".f", defaultValues.f)
        );
    }

}
