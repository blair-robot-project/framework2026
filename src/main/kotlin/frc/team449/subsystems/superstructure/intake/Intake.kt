package frc.team449.subsystems.superstructure.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.simulation.MockLaserCan
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake(
  private val topMotor: TalonFX,
  private val rightMotor: TalonFX,
  private val leftMotor: TalonFX,
  private val bottomCoralSensor: LaserCanInterface,
  private val leftCoralSensor: LaserCanInterface,
  private val rightCoralSensor: LaserCanInterface,
  private val topCoralSensor: LaserCanInterface
) : SubsystemBase() {

  // TODO: PID controls for each separate motor
  private val topController = PIDController(2.1778, 0.0, 0.010)
  private val rightController = PIDController(2.1778, 0.0, 0.010)
  private val leftController = PIDController(2.1778, 0.0, 0.010)

  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setControl(VoltageOut(voltage)) }
    }
  }

  fun setVoltageTop(voltage: Double) = setVoltage(topMotor, voltage = voltage)
  fun setVoltageRight(voltage: Double) = setVoltage(rightMotor, voltage = voltage)
  fun setVoltageLeft(voltage: Double) = setVoltage(leftMotor, voltage = voltage)
  fun setVoltageSides(voltage: Double) = setVoltage(rightMotor, leftMotor, voltage = voltage)

  fun intakeCoralHorizontal(): Command {
    return run {
      setVoltageTop(IntakeConstants.CORAL_INTAKE_VOLTAGE)
      setVoltageSides(IntakeConstants.CORAL_INTAKE_VOLTAGE) // change to some lower voltage
    }.until { coralHorizontal() }
      .andThen(holdCoral())
  }

  fun intakeCoralVertical(): Command {
    return run {
      setVoltageSides(IntakeConstants.CORAL_INTAKE_VOLTAGE)
      setVoltageTop(IntakeConstants.CORAL_INTAKE_VOLTAGE) // lower voltage
    }.until { coralVertical() }
      .andThen(holdCoral())
  }

  /*fun holdCoral(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position
    }
      .andThen(InstantCommand({ motor.setVoltage(2.0) }))
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }*/

  data class motorControl(
    val motor: TalonFX,
    val controller: PIDController
  )

  val motorControls = listOf(
    motorControl(topMotor, topController),
    motorControl(rightMotor, rightController),
    motorControl(leftMotor, leftController)
  )

  fun holdCoral(): Command {
    return runOnce {
      motorControls.forEach {
        it.controller.reset()
        it.controller.setpoint = it.motor.position.valueAsDouble
      }
    }
      .andThen(
        InstantCommand({
          motorControls.forEach {
            it.motor.setVoltage(2.0)
          } 
        })
      )
      .andThen(stop())
      .andThen(
        run {
          motorControls.forEach { it.motor.setVoltage(it.controller.calculate(it.motor.position.valueAsDouble)) }
        }
      )
  }

  /*fun holdCoralForward(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position + 1.0
    }
      .andThen(InstantCommand({ motor.setVoltage(2.0) }))
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }*/

  fun holdCoralForward(): Command {
    return runOnce {
      motorControls.forEach {
        it.controller.reset()
        it.controller.setpoint = it.motor.position.valueAsDouble + 1.0
      }
    }
      .andThen(
        InstantCommand({
          motorControls.forEach {
            it.motor.setVoltage(2.0)
          }
        })
      )
      .andThen(stop())
      .andThen(
        run {
          motorControls.forEach {
            it.motor.setVoltage(it.controller.calculate(it.motor.position.valueAsDouble))
          }
        }
      )
  }

  fun intakeAlgae(): Command {
    return setVoltageTop(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
  }

  fun holdAlgae(): Command {
    return setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE)
  }

  fun descoreAlgae(): Command {
    return setVoltageTop(IntakeConstants.DESCORE_ALGAE_VOLTAGE)
  }

  fun outtakeL1(): Command {
    return setVoltageTop(IntakeConstants.L1_OUTTAKE)
  }

  fun outtakeCoral(): Command {
    return setVoltageSides(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
  }

  fun outtakeCoralPivot(): Command {
    return setVoltageSides(IntakeConstants.CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE)
  }

  fun outtakeAlgae(): Command {
    return setVoltageTop(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
  }

  private fun laserCanDetected(laserCan: LaserCanInterface): Boolean {
    val measurement: LaserCanInterface.Measurement = laserCan.measurement
    if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
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

  // Coral is controlled by the Intake
  fun coralControlled(): Boolean {
    return coralVertical() || coralHorizontal()
  }

  // Coral is not vertical or horizontal but is detected by one of the sensors
  fun coralMisplaced(): Boolean {
    return coralDetected() && !coralVertical() && !coralHorizontal()
  }

  // What is the point of this
  fun algaeDetected(): Boolean {
    return false
  }

  fun stop(): Command {
    return runOnce {
      topMotor.stopMotor()
      rightMotor.stopMotor()
      leftMotor.stopMotor()
    }
  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    // DogLog.log("Intake/Motor Voltage", motor.appliedOutput * 12.0)
    // DogLog.log("Intake/Motor Position", motor.encoder.position)

    DogLog.log("Intake/LaserCan/Bottom Sensor Distance (mm)", bottomCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Left Sensor Distance (mm)", leftCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Right Sensor Distance (mm)", rightCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Top Sensor Distance (mm)", topCoralSensor.measurement.distance_mm.toDouble())
  }

  companion object {
    fun createIntake(): Intake {
      /*val motor = createSparkMax(
        id = IntakeConstants.MOTOR_ID,
        inverted = IntakeConstants.MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )*/

      val topMotor = TalonFX(IntakeConstants.TOP_MOTOR_ID)
      val rightMotor = TalonFX(IntakeConstants.LEFT_MOTOR_ID)
      val leftMotor = TalonFX(IntakeConstants.RIGHT_MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = IntakeConstants.INVERTED
      config.MotorOutput.NeutralMode = IntakeConstants.BRAKE_MODE

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_LIM
      config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_LIM

      // Use MockLaserCan in Simulation
      if (isSimulation()) {
        return Intake(
          topMotor,
          leftMotor,
          rightMotor,
          MockLaserCan(),
          MockLaserCan(),
          MockLaserCan(),
          MockLaserCan()
        )
      }

      return Intake(
        topMotor,
        rightMotor,
        leftMotor,
        LaserCan(IntakeConstants.BOTTOM_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.LEFT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.RIGHT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.TOP_CORAL_SENSOR_CAN_ID)
      )
    }
  }
}
