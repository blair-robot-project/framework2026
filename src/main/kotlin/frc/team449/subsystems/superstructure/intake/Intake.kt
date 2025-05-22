package frc.team449.subsystems.superstructure.intake

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createSparkMax

class Intake(
  private val motor: SparkMax,
  private val coralInfrared: DigitalInput,
  private val leftCoralInfrared: DigitalInput,
  private val rightCoralInfrared: DigitalInput,
  private val topCoralInfrared: DigitalInput
) : SubsystemBase() {

  private val controller = PIDController(2.1778, 0.0, 0.010)

  fun setVoltage(voltage: Double): Command {
    return runOnce {
      motor.setVoltage(voltage)
    }
  }

  fun intakeCoral(): Command {
    return setVoltage(IntakeConstants.CORAL_INTAKE_VOLTAGE)
  }

  fun holdCoral(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position
    }
      .andThen(InstantCommand({ motor.setVoltage(2.0) }))
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }

  fun holdCoralForward(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position + 1.0
    }
      .andThen(InstantCommand({ motor.setVoltage(2.0) }))
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }

  fun intakeAlgae(): Command {
    return setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
  }

  fun holdAlgae(): Command {
    return setVoltage(IntakeConstants.ALGAE_HOLD_VOLTAGE)
  }

  fun descoreAlgae(): Command {
    return setVoltage(IntakeConstants.DESCORE_ALGAE_VOLTAGE)
  }

  fun outtakeL1(): Command {
    return setVoltage(IntakeConstants.L1_OUTTAKE)
  }

  fun outtakeCoral(): Command {
    return setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
  }

  fun outtakeCoralPivot(): Command {
    return setVoltage(IntakeConstants.CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE)
  }

  fun outtakeAlgae(): Command {
    return setVoltage(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
  }

  fun coralDetected(): Boolean {
    return !coralInfrared.get()
  }

  fun coralNotDetected(): Boolean {
    return coralInfrared.get()
  }

  fun coralVertical(): Boolean {
    return !coralInfrared.get() && !topCoralInfrared.get()
  }

  fun coralHorizontal(): Boolean {
    return !coralInfrared.get() && !leftCoralInfrared.get() && !rightCoralInfrared.get()
  }

  // Coral is not vertical or horizontal but is detected by one of the sensors
  fun coralMisplaced(): Boolean {
    return !coralVertical() && !coralHorizontal() && (coralInfrared.get() || leftCoralInfrared.get() || rightCoralInfrared.get() || topCoralInfrared.get())
  }

  fun algaeDetected(): Boolean {
    // What is the point of this
    return false
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
    DogLog.log("Intake/Motor Voltage", motor.appliedOutput * 12.0)
    DogLog.log("Intake/Motor Position", motor.encoder.position)
    DogLog.log("Intake/Coral IR sensor", !coralInfrared.get())
    DogLog.log("Intake/Left Coral IR sensor", !leftCoralInfrared.get())
    DogLog.log("Intake/Right Coral IR sensor", !rightCoralInfrared.get())
    DogLog.log("Intake/Top Coral IR sensor", !topCoralInfrared.get())
  }

  companion object {
    fun createIntake(): Intake {
      val motor = createSparkMax(
        id = IntakeConstants.MOTOR_ID,
        inverted = IntakeConstants.MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )

      val coralSensor = DigitalInput(IntakeConstants.CORAL_SENSOR_DIO_PORT)
      val leftCoralSensor = DigitalInput(IntakeConstants.LEFT_CORAL_SENSOR_DIO_PORT)
      val rightCoralSensor = DigitalInput(IntakeConstants.RIGHT_CORAL_SENSOR_DIO_PORT)
      val topCoralSensor = DigitalInput(IntakeConstants.TOP_CORAL_SENSOR_DIO_PORT)
      return Intake(motor, coralSensor, leftCoralSensor, rightCoralSensor, topCoralSensor)
    }
  }
}
