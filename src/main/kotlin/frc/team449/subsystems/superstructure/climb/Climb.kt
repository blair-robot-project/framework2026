package frc.team449.subsystems.superstructure.climb

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.system.motor.KrakenDogLog

// TODO(the entire class bru)
class Climb(
  private val motor: TalonFX
//  private val sensor: DigitalInput
) : SubsystemBase() {

  fun runClimbWheels(): Command {
    return runOnce {
      motor.setVoltage(ClimbConstants.RUN_VOLTAGE)
    }
  }

  fun holdClimbWheels(): Command {
    return runOnce {
      motor.setVoltage(ClimbConstants.HOLD_VOLTAGE)
    }
  }

//  fun cageDetected(): Boolean {
//    return !sensor.get()
//  }

  fun waitUntilCurrentSpike(): Command {
    return WaitCommand(0.1750)
//      .andThen(WaitUntilCommand { motor.outputCurrent > 55.0 })
      .andThen(WaitUntilCommand { motor.supplyCurrent.valueAsDouble > 55.0 })
  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    KrakenDogLog.log("Intake/topMotor", motor)
  }

  companion object {
    fun createClimb(): Climb {
      val motor = TalonFX(ClimbConstants.MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = ClimbConstants.INVERTED
      config.MotorOutput.NeutralMode = ClimbConstants.BRAKE_MODE

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_LIM
      config.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_LIM

//      val sensor = DigitalInput(ClimbConstants.SENSOR_DIO_PORT)

      return Climb(motor) // , sensor)
    }
  }
}
