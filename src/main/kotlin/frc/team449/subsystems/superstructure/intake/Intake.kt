package frc.team449.subsystems.superstructure.intake

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createSparkMax
import kotlin.math.log

// TODO(the entire class bru)
class Intake(
  private val motor: SparkMax,
  private val infrared: DigitalInput
) : SubsystemBase() {

  fun intakeCoral(): Command {
    return runOnce {
      motor.setVoltage(IntakeConstants.CORAL_INTAKE_VOLTAGE)
    }
  }

  fun holdCoral(): Command {
    return runOnce {
      motor.setVoltage(IntakeConstants.CORAL_HOLD_VOLTAGE)
    }
  }

  fun coralDetected(): Boolean {
    return !infrared.get()
  }

//  fun intakeAlgae(): Command {
//    return runOnce {
//      frontMotor.setVoltage(-IntakeConstants.ALGAE_INTAKE_VOLTAGE)
//      backMotor.setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
//    }
//  }
//
//  fun outtakeAlgae(): Command {
//    return runOnce {
//      frontMotor.setVoltage(-IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
//      backMotor.setVoltage(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
//    }
//  }

  fun outtakeCoral(): Command {
    return runOnce {
      motor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
    }
  }

//  fun intakeAlgae(): Command {
//    return runOnce {
//      frontMotor.setVoltage(-IntakeConstants.ALGAE_INTAKE_VOLTAGE)
//      backMotor.setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
//    }
//  }
//
//  fun outtakeAlgae(): Command {
//    return runOnce {
//      frontMotor.setVoltage(-IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
//      backMotor.setVoltage(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
//    }
//  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    DogLog.log("Intake/Motor Voltage", motor.appliedOutput * 12.0)
    DogLog.log("Intake/IR sensor", !infrared.get())
  }

  companion object {
    fun createIntake(): Intake {
      val motor = createSparkMax(
        id = IntakeConstants.FRONT_MOTOR_ID,
        inverted = IntakeConstants.FRONT_MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )

      val sensor = DigitalInput(IntakeConstants.SENSOR_DIO_PORT)

      return Intake(motor, sensor)
    }
  }
}
