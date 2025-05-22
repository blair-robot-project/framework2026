package frc.team449.subsystems.superstructure.intake

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createSparkMax
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

class Intake(
  private val motor: SparkMax,
  private val bottomCoralSensor: LaserCan,
  private val leftCoralSensor: LaserCan,
  private val rightCoralSensor: LaserCan,
  private val topCoralSensor: LaserCan
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

  fun laserCanDetected(laserCan: LaserCan): Boolean {
    val measurement: LaserCan.Measurement = laserCan.getMeasurement()
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      if (measurement.distance_mm <= IntakeConstants.LASER_CAN_SENSOR_MIN_DISTANCE_MM) {
        return true
      }
    }
    return false
  }

  fun coralDetected(): Boolean {
    return laserCanDetected(bottomCoralSensor) || laserCanDetected(leftCoralSensor) || laserCanDetected(rightCoralSensor) || laserCanDetected(topCoralSensor)
  }

  fun coralNotDetected(): Boolean {
    return !coralDetected()
  }

  fun coralVertical(): Boolean {
    return laserCanDetected(bottomCoralSensor) && laserCanDetected(topCoralSensor) && !laserCanDetected(leftCoralSensor) && !laserCanDetected(rightCoralSensor)
  }

  fun coralHorizontal(): Boolean {
    return laserCanDetected(bottomCoralSensor) && laserCanDetected(leftCoralSensor) && laserCanDetected(rightCoralSensor) && !laserCanDetected(topCoralSensor)
  }

  // Coral is not vertical or horizontal but is detected by one of the sensors
  fun coralMisplaced(): Boolean {
    return coralDetected() && !coralVertical() && !coralHorizontal()
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
    DogLog.log("Intake/Bottom Coral LaserCAN Distance (mm)", bottomCoralSensor.getMeasurement().distance_mm)
    DogLog.log("Intake/Left Coral LaserCAN Distance (mm)", leftCoralSensor.getMeasurement().distance_mm)
    DogLog.log("Intake/Right Coral LaserCAN Distance (mm)", rightCoralSensor.getMeasurement().distance_mm)
    DogLog.log("Intake/Top Coral LaserCAN Distance (mm)", topCoralSensor.getMeasurement().distance_mm)
  }

  companion object {
    fun createIntake(): Intake {
      val motor = createSparkMax(
        id = IntakeConstants.MOTOR_ID,
        inverted = IntakeConstants.MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )

      val coralSensor = LaserCan(IntakeConstants.BOTTOM_CORAL_SENSOR_CAN_ID)
      val leftCoralSensor = LaserCan(IntakeConstants.LEFT_CORAL_SENSOR_CAN_ID)
      val rightCoralSensor = LaserCan(IntakeConstants.RIGHT_CORAL_SENSOR_CAN_ID)
      val topCoralSensor = LaserCan(IntakeConstants.TOP_CORAL_SENSOR_CAN_ID)
      return Intake(motor, coralSensor, leftCoralSensor, rightCoralSensor, topCoralSensor)
    }
  }
}
