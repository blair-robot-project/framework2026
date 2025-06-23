package frc.team449.subsystems.superstructure.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.simulation.MockLaserCan
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
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
  private val backCoralSensor: LaserCanInterface,
  private val leftCoralSensor: LaserCanInterface,
  private val rightCoralSensor: LaserCanInterface,
  private val middleCoralSensor: LaserCanInterface
) : SubsystemBase() {

  private val sensors = listOf(
    backCoralSensor,
    leftCoralSensor,
    rightCoralSensor,
    middleCoralSensor
  )

  private var gamePiece = Piece.CORAL

  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setControl(VoltageOut(voltage)) }
    }
  }

  private var sensorInitFailed = false
  private var failedSensors = listOf(sensors)

  init {
    for (sensor in sensors) {
      try {
        sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
        sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
        sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
      } catch (_: Exception) {
        sensorInitFailed = true
        failedSensors.indexOf(sensors)
      }
    }
  }

  private fun setVoltageTop(voltage: Double): Command { return setVoltage(topMotor, voltage = voltage) }
  private fun setVoltageSides(voltage: Double): Command { return setVoltage(rightMotor, leftMotor, voltage = voltage) }

  private fun moveCoralRight(): Command {
    return runOnce {
      rightMotor.setVoltage(-IntakeConstants.RUN_SIDES)
      leftMotor.setVoltage(IntakeConstants.RUN_SIDES)
    }
  }

  private fun moveCoralLeft(): Command {
    return runOnce {
      rightMotor.setVoltage(IntakeConstants.RUN_SIDES)
      leftMotor.setVoltage(-IntakeConstants.RUN_SIDES)
    }
  }

  fun inwards(): Command {
    return runOnce {
      topMotor.setVoltage(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
      rightMotor.setVoltage(IntakeConstants.SIDE_ROLLER_IN_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.SIDE_ROLLER_IN_VOLTAGE)
    }
  }

  private fun topRollerIn(): Command {
    return runOnce {
      topMotor.setVoltage(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
    }
  }

  private fun outwards(): Command {
    return runOnce {
      topMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      rightMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
    }
  }

  fun centerHorizontally(): Command {
    return Commands.sequence(
      moveCoralLeft(),
      WaitUntilCommand { !rightSensorDetected() },
      moveCoralRight(),
      WaitCommand(0.1),
      holdCoral()
    )
  }

  private var unmiddling = false
  fun intakeToHorizontal(): Command {
    changePieceToCoral().schedule()
    return Commands.sequence(
      topRollerIn().until { coralDetected() },
      FunctionalCommand(
        { unmiddling = false },
        {
          if (rightSensorDetected()) {
            if (!unmiddling) {
              // move left
              rightMotor.setVoltage(IntakeConstants.RUN_SIDES)
              leftMotor.setVoltage(-IntakeConstants.RUN_SIDES)
            } else {
              // move right until just right sensor
              rightMotor.setVoltage(-IntakeConstants.RUN_SIDES)
              leftMotor.setVoltage(IntakeConstants.RUN_SIDES)
              if (onlyRightSensor()) {
                unmiddling = false
              }
            }
          }
          if (leftSensorDetected()) {
            // move right
            rightMotor.setVoltage(-IntakeConstants.RUN_SIDES)
            leftMotor.setVoltage(IntakeConstants.RUN_SIDES)
          }
          if (onlyMiddleSensor()) {
            // move right and out slowly
            rightMotor.setVoltage(-IntakeConstants.RUN_SIDES)
            leftMotor.setVoltage(IntakeConstants.RUN_SIDES)
            topMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE / 2)
            unmiddling = true
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
          rightMotor.setVoltage(-IntakeConstants.RUN_SIDES)
          leftMotor.setVoltage(IntakeConstants.RUN_SIDES)
        } else {
          // inwards
          topMotor.setVoltage(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
          rightMotor.setVoltage(IntakeConstants.SIDE_ROLLER_IN_VOLTAGE)
          leftMotor.setVoltage(IntakeConstants.SIDE_ROLLER_IN_VOLTAGE)
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

  fun holdCoral(): Command {
    return stop().andThen(
      run {
        topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
      }
    )
      .andThen(stop())
  }

  fun holdCoralToFront(): Command {
    return Commands.sequence(
      outwards().onlyWhile { backSensorDetected() },
      holdCoral(),
      WaitCommand(0.25),
      inwards().until { middleSensorDetected() && backSensorDetected() }.withTimeout(0.3),
      holdCoral()
    )
  }

  fun holdCoralToPivot(): Command {
    return Commands.sequence(
      inwards().onlyWhile { middleSensorDetected() },
      holdCoral(),
      WaitCommand(0.5),
      outwards().until { middleSensorDetected() && backSensorDetected() }.withTimeout(0.2),
      holdCoral()
    )
  }

  fun holdCoralForward(): Command {
    return stop().andThen(
      runOnce {
        topMotor.setControl(PositionDutyCycle(topMotor.position.valueAsDouble + 1))
        rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble + 1))
        leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble + 1))
      }.andThen(
        WaitUntilCommand {
          // velocities are in rps
          topMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            rightMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            leftMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY
        }
      ).andThen(holdCoral())
    )
  }

  fun holdCoralForwardAuto(): Command {
    return stop().andThen(
      runOnce {
        topMotor.setControl(PositionDutyCycle(topMotor.position.valueAsDouble + 1.875))
        rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble + 1.875))
        leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble + 1.875))
      }.andThen(
        WaitUntilCommand {
          // velocities are in rps
          topMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            rightMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            leftMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY
        }
      ).andThen(holdCoral())
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
    return setVoltageTop(IntakeConstants.L1_OUTTAKE)
  }

  fun outtakeCoral(): Command {
    changePieceToNone(true).schedule()
    return setVoltageSides(IntakeConstants.CORAL_OUTTAKE_VOLTAGE).andThen(setVoltageTop(IntakeConstants.CORAL_OUTTAKE_VOLTAGE))
  }

  fun outtakeCoralPivot(): Command {
    changePieceToNone(true).schedule()
    return runOnce {
      setVoltageSides(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      setVoltageTop(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
    }
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

  private fun laserCanUnplugged(laserCan: LaserCanInterface): Boolean {
    val measurement = laserCan.measurement
    return measurement == null // returns TRUE if the laserCan is unplugged
  }

  fun coralDetected(): Boolean {
    return laserCanDetected(backCoralSensor) || laserCanDetected(leftCoralSensor) || laserCanDetected(rightCoralSensor) || laserCanDetected(middleCoralSensor)
  }

  fun coralNotDetected(): Boolean {
    return !coralDetected()
  }

  private fun coralIsVertical(): Boolean {
    return laserCanDetected(backCoralSensor) && laserCanDetected(middleCoralSensor) && !laserCanDetected(leftCoralSensor) && !laserCanDetected(rightCoralSensor)
  }

  private fun coralIsHorizontal(): Boolean {
    return !laserCanDetected(backCoralSensor) && laserCanDetected(leftCoralSensor) && laserCanDetected(rightCoralSensor) && laserCanDetected(middleCoralSensor)
  }

  private fun rightSensorDetected(): Boolean {
    return laserCanDetected(rightCoralSensor)
  }

  private fun leftSensorDetected(): Boolean {
    return laserCanDetected(leftCoralSensor)
  }

  private fun middleSensorDetected(): Boolean {
    return laserCanDetected(middleCoralSensor)
  }

  private fun backSensorDetected(): Boolean {
    return laserCanDetected(backCoralSensor)
  }

  private fun onlyRightSensor(): Boolean {
    return !laserCanDetected(leftCoralSensor) && laserCanDetected(rightCoralSensor) && !laserCanDetected(middleCoralSensor)
  }

  private fun onlyLeftSensor(): Boolean {
    return laserCanDetected(leftCoralSensor) && !laserCanDetected(rightCoralSensor) && !laserCanDetected(middleCoralSensor)
  }

  private fun onlyMiddleSensor(): Boolean {
    return !laserCanDetected(leftCoralSensor) && !laserCanDetected(rightCoralSensor) && laserCanDetected(middleCoralSensor)
  }

  fun algaeDetected(): Boolean {
    return gamePiece == Piece.ALGAE
  }

  private fun changePieceToAlgae(): Command {
    return WaitUntilCommand {
      topMotor.motorVoltage.valueAsDouble >
        IntakeConstants.ALGAE_STALL_THRESHOLD
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
              IntakeConstants.ALGAE_STALL_THRESHOLD
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
    val back: LaserCanInterface.Measurement? = backCoralSensor.measurement
    val left: LaserCanInterface.Measurement? = leftCoralSensor.measurement
    val right: LaserCanInterface.Measurement? = rightCoralSensor.measurement
    val middle: LaserCanInterface.Measurement? = middleCoralSensor.measurement
    DogLog.log("Intake/LaserCan/Back Sensor Distance (mm)", back?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Left Sensor Distance (mm)", left?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Right Sensor Distance (mm)", right?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Middle Sensor Distance (mm)", middle?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/ Back sensor", laserCanDetected(backCoralSensor))
    DogLog.log("Intake/ Right sensor", laserCanDetected(rightCoralSensor))
    DogLog.log("Intake/ Left sensor", laserCanDetected(leftCoralSensor))
    DogLog.log("Intake/ Middle sensor", laserCanDetected(middleCoralSensor))
    DogLog.log("Intake/ sensorConfig", sensorInitFailed)
    DogLog.log("Intake/ Middle sensor state", laserCanUnplugged(middleCoralSensor))
    DogLog.log("Intake/ Right sensor state", laserCanUnplugged(rightCoralSensor))
    DogLog.log("Intake/ Left sensor state", laserCanUnplugged(leftCoralSensor))
    DogLog.log("Intake/ Back sensor state", laserCanUnplugged(backCoralSensor))
    val pieceName = when (gamePiece) {
      Piece.NONE -> "none"
      Piece.ALGAE -> "algae"
      Piece.CORAL -> "coral"
    }
    DogLog.log("Intake/Piece", pieceName)
    DogLog.log("Intake/Stall Current", topMotor.motorStallCurrent.valueAsDouble)
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
