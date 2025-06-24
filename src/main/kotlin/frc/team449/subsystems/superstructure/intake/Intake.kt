package frc.team449.subsystems.superstructure.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.simulation.MockLaserCan
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.system.motor.KrakenDogLog

enum class Piece {
  CORAL,
  ALGAE,
  NONE
}

class Intake(
  private val topMotor: TalonFX, // kraken x60
  private val rightMotor: TalonFX, // kraken x44
  private val leftMotor: TalonFX, // kraken x44
  private val backSensor: LaserCanInterface,
  private val leftSensor: LaserCanInterface,
  private val rightSensor: LaserCanInterface,
  private val middleSensor: LaserCanInterface
) : SubsystemBase() {

  private val sensors = listOf(
    backSensor,
    leftSensor,
    rightSensor,
    middleSensor
  )

  private var gamePiece = Piece.CORAL

  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setVoltage(voltage) }
    }
  }

  private var allSensorsConfigured = true
  private var lasercanConfigured = listOf<Boolean>()

  init {
    for (sensor in sensors) {
      try {
        sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
        sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
        sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
        lasercanConfigured.plus(true)
      } catch (_: Exception) {
        lasercanConfigured.plus(false)
        allSensorsConfigured = false
      }
    }
  }

  private fun setVoltageTop(voltage: Double): Command { return setVoltage(topMotor, voltage = voltage) }
  private fun setVoltageSides(voltage: Double): Command { return setVoltage(rightMotor, leftMotor, voltage = voltage) }

  private fun moveCoralRight(): Command {
    return runOnce {
      rightMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
      leftMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
    }
  }

  private fun moveCoralLeft(): Command {
    return runOnce {
      rightMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
    }
  }

  fun inwards(): Command {
    return runOnce {
      topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
      rightMotor.setVoltage(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE)
    }
  }

  private fun outwards(): Command {
    return runOnce {
      topMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      rightMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
    }
  }

  fun holdCoral(): Command {
    return stop().andThen(
      runOnce {
        topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
      }
    )
  }

  fun holdCoralOppSide(): Command {
    return stop().andThen(
      runOnce {
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble - 1))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble - 1))
      }.andThen(
        WaitUntilCommand {
          // velocities are in rps
          rightMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
          leftMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY
        }
      ).andThen(holdCoral())
    )
  }

  fun holdCoralPivotSide(): Command {
    return stop().andThen(
      runOnce {
        rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble + 1))
        leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble + 1))
      }.andThen(
        WaitUntilCommand {
          // velocities are in rps
          rightMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            leftMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY
        }
      ).andThen(holdCoral())
    )
  }

  fun centerCoralHorizontally(): Command {
    return Commands.sequence(
      runOnce { topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / 10) },
      moveCoralLeft(),
      WaitUntilCommand { !rightSensorDetected() },
      runOnce {
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble + 0.5))
        leftMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble - 0.5))
      }.andThen(
        WaitUntilCommand {
          // velocities are in rps
          rightMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            leftMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY
        }
      ),
      holdCoral()
    )
  }

  private var unverticaling = false
  fun intakeToHorizontal(): Command {
    changePieceToCoral().schedule()
    return Commands.sequence(
      runOnce { topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE) },
      WaitUntilCommand { coralDetected() },
      FunctionalCommand(
        { unverticaling = false },
        {
          if (rightSensorDetected()) {
            if (!unverticaling) {
              // move left
              rightMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
              leftMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
            } else {
              if (onlyRightSensor()) {
                unverticaling = false
              } else {
                // move right until just right sensor
                topMotor.setVoltage(-IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / 10)
                rightMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
                leftMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
              }
            }
          }
          if (leftSensorDetected()) {
            // move right
            rightMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
            leftMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
          }
          if (onlyMiddleSensor()) {
            if(!backSensorDetected()) {
              if(unverticaling) {
                // move right and out slowly
                rightMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
                leftMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
                topMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE / 2)
              } else {
                //run inwards till we hit back sensor
                topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
                rightMotor.setVoltage(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE)
                leftMotor.setVoltage(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE)
              }
            } else { // hitting back sensor
              unverticaling = true
              // move right and out slowly
              rightMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
              leftMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
              topMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE / 3)
            }
          }
          if (coralNotDetected()) {
            topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
          }
        },
        {
          // stop at end
          topMotor.stopMotor()
          rightMotor.stopMotor()
          leftMotor.stopMotor()
        },
        { coralIsHorizontal() }
      ),
    )
  }

  fun intakeToVertical(): Command {
    changePieceToCoral().schedule()
    return FunctionalCommand(
      { },
      {
        if (leftSensorDetected() && rightSensorDetected()) {
          // run to a side
          rightMotor.setVoltage(-IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
          leftMotor.setVoltage(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE)
        } else {
          // inwards
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
          rightMotor.setVoltage(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE)
          leftMotor.setVoltage(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE)
        }
      },
      {
        // stop at end
        topMotor.stopMotor()
        rightMotor.stopMotor()
        leftMotor.stopMotor()
      },
      { backSensorDetected() },
    )
  }

  fun intakeAlgae(): Command {
    changePieceToAlgae().schedule()
    return setVoltageTop(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
  }

  fun holdAlgae(): Command {
    return setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE)
  }

  fun outtakeL1(): Command {
    changePieceToNone(true).schedule()
    return setVoltageTop(IntakeConstants.TOP_L1_OUTTAKE)
  }

  fun outtakeCoral(): Command {
    changePieceToNone(true).schedule()
    return setVoltageSides(IntakeConstants.SIDES_CORAL_OUTTAKE_VOLTAGE)
      .andThen(setVoltageTop(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE))
  }

  fun outtakeCoralPivot(): Command {
    changePieceToNone(true).schedule()
    return setVoltageSides(IntakeConstants.SIDES_CORAL_OUTTAKE_VOLTAGE)
      .andThen(setVoltageTop(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE))
  }

  fun outtakeAlgae(): Command {
    changePieceToNone(false).schedule()
    return setVoltageTop(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
  }

  private fun laserCanDetected(laserCan: LaserCanInterface): Boolean {
    val measurement: LaserCanInterface.Measurement? = laserCan.measurement
    return measurement != null && (
      measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
        measurement.distance_mm <= IntakeConstants.CORAL_DETECTION_THRESHOLD
      )
  }

  private fun laserCanIsPlugged(laserCan: LaserCanInterface): Boolean {
    val measurement = laserCan.measurement
    if (measurement == null){ return false } else{ return true }
  }

  fun coralDetected(): Boolean {
    return laserCanDetected(backSensor) || laserCanDetected(leftSensor) || laserCanDetected(rightSensor) || laserCanDetected(middleSensor)
  }

  fun coralNotDetected(): Boolean {
    return !coralDetected()
  }

  private fun coralIsVertical(): Boolean {
    return laserCanDetected(backSensor) && laserCanDetected(middleSensor) && !laserCanDetected(leftSensor) && !laserCanDetected(rightSensor)
  }

  private fun coralIsHorizontal(): Boolean {
    return !laserCanDetected(backSensor) && laserCanDetected(leftSensor) && laserCanDetected(rightSensor) && laserCanDetected(middleSensor)
  }

  private fun rightSensorDetected(): Boolean {
    return laserCanDetected(rightSensor)
  }

  private fun leftSensorDetected(): Boolean {
    return laserCanDetected(leftSensor)
  }

  private fun middleSensorDetected(): Boolean {
    return laserCanDetected(middleSensor)
  }

  private fun backSensorDetected(): Boolean {
    return laserCanDetected(backSensor)
  }

  private fun onlyRightSensor(): Boolean {
    return !laserCanDetected(leftSensor) && laserCanDetected(rightSensor) && !laserCanDetected(middleSensor)
  }

  private fun onlyLeftSensor(): Boolean {
    return laserCanDetected(leftSensor) && !laserCanDetected(rightSensor) && !laserCanDetected(middleSensor)
  }

  private fun onlyMiddleSensor(): Boolean {
    return !laserCanDetected(leftSensor) && !laserCanDetected(rightSensor) && laserCanDetected(middleSensor)
  }

  fun algaeDetected(): Boolean {
    return gamePiece == Piece.ALGAE
  }

  private fun changePieceToAlgae(): Command {
    return WaitUntilCommand {
      topMotor.motorVoltage.valueAsDouble >
        IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD
    }.onlyIf { RobotBase.isReal() }
      .andThen(runOnce { gamePiece = Piece.ALGAE })
  }

  private fun changePieceToCoral(): Command {
    return WaitUntilCommand { coralDetected() }.onlyIf { RobotBase.isReal() }
      .andThen(runOnce { gamePiece = Piece.CORAL })
  }

  private fun changePieceToNone(coralOuttaken: Boolean): Command {
    return Commands.sequence(
      ConditionalCommand(
        WaitUntilCommand { !coralDetected() },
        WaitCommand(0.1).andThen(
          WaitUntilCommand {
            topMotor.motorStallCurrent.valueAsDouble <
              IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD
          }
        )
      ) { coralOuttaken },
      runOnce { gamePiece = Piece.NONE }
    )
  }

  fun resetPiece(): Command {
    return runOnce {
      gamePiece = Piece.CORAL
    }
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
    KrakenDogLog.log("Intake/topMotor", topMotor)
    KrakenDogLog.log("Intake/rightMotor", rightMotor)
    KrakenDogLog.log("Intake/leftMotor", leftMotor)

    DogLog.log("Intake/HorizontalIntake", coralIsHorizontal())
    DogLog.log("Intake/VerticalIntake", coralIsVertical())

    val back: LaserCanInterface.Measurement? = backSensor.measurement
    val left: LaserCanInterface.Measurement? = leftSensor.measurement
    val right: LaserCanInterface.Measurement? = rightSensor.measurement
    val middle: LaserCanInterface.Measurement? = middleSensor.measurement
    DogLog.log("Intake/LaserCan/Distance(mm)/Back Sensor", back?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Left Sensor", left?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Right Sensor", right?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Middle Sensor", middle?.distance_mm?.toDouble() ?: -1.0)

    DogLog.log("Intake/LaserCan/ Detecting/ Back", laserCanDetected(backSensor))
    DogLog.log("Intake/LaserCan/ Detecting/ Right", laserCanDetected(rightSensor))
    DogLog.log("Intake/LaserCan/ Detecting/ Left", laserCanDetected(leftSensor))
    DogLog.log("Intake/LaserCan/ Detecting/ Middle", laserCanDetected(middleSensor))

    DogLog.log("Intake/LaserCan/ Connection/ Middle", laserCanIsPlugged(middleSensor))
    DogLog.log("Intake/LaserCan/ Connection/ Right", laserCanIsPlugged(rightSensor))
    DogLog.log("Intake/LaserCan/ Connection/ Left", laserCanIsPlugged(leftSensor))
    DogLog.log("Intake/LaserCan/ Connection/ Back", laserCanIsPlugged(backSensor))

    DogLog.log("Intake/LaserCan/All Sensors Configured",allSensorsConfigured)
    DogLog.log("Intake/LaserCan/ Configured list",lasercanConfigured.toBooleanArray())

    val pieceName = when (gamePiece) {
      Piece.NONE -> "none"
      Piece.ALGAE -> "algae"
      Piece.CORAL -> "coral"
    }
    DogLog.log("Intake/Piece", pieceName)
    DogLog.log("Intake/Voltage", topMotor.motorVoltage.valueAsDouble)
  }

  companion object {
    fun createIntake(): Intake {
      val topMotor = TalonFX(IntakeConstants.TOP_MOTOR_ID)
      val rightMotor = TalonFX(IntakeConstants.LEFT_MOTOR_ID)
      val leftMotor = TalonFX(IntakeConstants.RIGHT_MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.NeutralMode = IntakeConstants.BRAKE_MODE

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_LIM
      config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_LIM

      config.MotorOutput.Inverted = IntakeConstants.TOP_INVERTED
      topMotor.configurator.apply(config)
      config.MotorOutput.Inverted = IntakeConstants.LEFT_INVERTED
      leftMotor.configurator.apply(config)
      config.MotorOutput.Inverted = IntakeConstants.RIGHT_INVERTED
      rightMotor.configurator.apply(config)

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
        LaserCan(IntakeConstants.BACK_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.LEFT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.RIGHT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.MIDDLE_CORAL_SENSOR_CAN_ID)
      )
    }
  }
}
