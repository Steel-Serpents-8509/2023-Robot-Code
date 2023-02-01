package org.firstinspires.ftc.teamcode

import android.annotation.SuppressLint
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.io.File
import java.util.Properties

object RobotProperties {
    @SuppressLint("SdCardPath")
    private val file = File("/sdcard/FIRST/robot.properties")
    private val properties: Properties = Properties()
    private var telemetry: Telemetry? = null

    init {
        if(file.exists() && file.isFile) {
            properties.load(file.inputStream())
        }
    }

    private fun getProperty(propertyName: String): String {
        return properties[propertyName] as String
    }

    private fun reportError(error: String) {
        telemetry?.addData("RobotProperties", error)
    }

    fun getIntLimit(limitName: String, default: Int): Int {
        val ret = getProperty("limits.$limitName").toIntOrNull()

        if(ret == null) {
            reportError("Limit not found: $limitName")
            return default
        }

        return ret
    }

    fun getIntPIDCoefficients(PIDName: String, default: PIDCoefficients): PIDCoefficients {
        return try {
            PIDCoefficients(getProperty("pid.$PIDName.p").toDouble(), getProperty("pid.$PIDName.i").toDouble(), getProperty("pid.$PIDName.d").toDouble())
        } catch (_: NumberFormatException) {
            reportError("Could not load PID values for: $PIDName")
            default
        }
    }

    fun getIntPIDCoefficients(PIDName: String): PIDCoefficients {
        return getIntPIDCoefficients(PIDName, PIDCoefficients(0.0, 0.0, 0.0))
    }

}