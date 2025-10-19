package frc.team449.util.customloggers

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.epilogue.CustomLoggerFor
import edu.wpi.first.epilogue.logging.ClassSpecificLogger
import edu.wpi.first.epilogue.logging.EpilogueBackend

@CustomLoggerFor(TalonFX::class)
class KrakenCustomLogger : ClassSpecificLogger<TalonFX>(TalonFX::class.java) {
  override fun update(backend: EpilogueBackend, motor: TalonFX) {
    backend.log("Motor Position", motor.position.value)
    backend.log("Motor Velocity", motor.velocity.value)
    backend.log("Motor Acceleration", motor.acceleration.value)
    backend.log("Closed Loop Reference", motor.closedLoopReference.valueAsDouble)
    backend.log("Closed Loop Error", motor.closedLoopError.value)
    backend.log("Closed Loop PID output", motor.closedLoopOutput.value)
    backend.log("Motor Voltage", motor.motorVoltage.value)
    backend.log("Supply Current", motor.supplyCurrent.value)
    backend.log("Stator Current", motor.statorCurrent.value)
    backend.log("Device Temperature", motor.deviceTemp.value)
  }
}