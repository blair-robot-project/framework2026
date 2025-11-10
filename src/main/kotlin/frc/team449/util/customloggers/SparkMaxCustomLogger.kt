package frc.team449.util.customloggers

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.CustomLoggerFor
import edu.wpi.first.epilogue.logging.ClassSpecificLogger
import edu.wpi.first.epilogue.logging.EpilogueBackend
import kotlin.jvm.java

@CustomLoggerFor(SparkMax::class)
class SparkMaxCustomLogger : ClassSpecificLogger<SparkMax>(SparkMax::class.java) {
  override fun update(backend: EpilogueBackend, motor: SparkMax) {
    backend.log("Motor Position", motor.encoder.position)
    backend.log("Motor Velocity", motor.encoder.velocity)
    backend.log("Motor Voltage", motor.busVoltage)
    backend.log("Output Current", motor.outputCurrent)
    backend.log("Device Temperature", motor.motorTemperature)
  }
}
