package frc.team449.subsystems.superstructure.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.simulation.MockLaserCan
import com.ctre.phoenix6.configs.TalonFXConfiguration
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

  private fun setMotorsRight(voltage: Double = IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE) {
    rightMotor.setVoltage(voltage)
    leftMotor.setVoltage(-voltage)
  }

  private fun setMotorsLeft(voltage: Double = IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE) {
    rightMotor.setVoltage(-voltage)
    leftMotor.setVoltage(voltage)
  }

  private fun setMotorsInwards(slowdownConstant: Double = 1.0) {
    topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
    setVoltageSides(IntakeConstants.SIDES_CORAL_INWARDS_VOLTAGE / slowdownConstant)
  }

  fun inwards(): Command {
    return runOnce {
      setMotorsInwards()
    }
  }

  private fun setMotorsOutwards(slowdownConstant: Double = 1.0) {
    topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / slowdownConstant)
    setVoltageSides(IntakeConstants.SIDES_CORAL_OUTTAKE_VOLTAGE / slowdownConstant)
  }

  private fun outwards(): Command {
    return runOnce {
      setMotorsOutwards()
    }
  }

  fun holdCoral(): Command {
    return stopMotors().andThen(
      runOnce {
        topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
      }
    )
  }

  fun holdCoralOppSide(): Command {
    return stopMotors().andThen(
      runOnce {
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble - IntakeConstants.HOLD_ANGLE_CHANGE))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble - IntakeConstants.HOLD_ANGLE_CHANGE))
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
    return stopMotors().andThen(
      runOnce {
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble + IntakeConstants.HOLD_ANGLE_CHANGE))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble + IntakeConstants.HOLD_ANGLE_CHANGE))
      }.andThen(
        WaitUntilCommand {
          // velocities are in rps
          rightMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY &&
            leftMotor.velocity.valueAsDouble < IntakeConstants.HOLDING_FINISH_VELOCITY
        }
      ).andThen(holdCoral())
    )
  }

  private var unverticaling = false
  private var coralIn = true
  fun intakeToHorizontal(): Command {
    return FunctionalCommand(
      {
        unverticaling = false
        coralIn = true // source: trust me bro
      },
      {
        if (coralNotDetected()) {
          // pull in coral until a sensor detects
          coralIn = false
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
        } else {
          // stop top motor once a coral is detected so we don't have it running when it shouldn't be
          if (!coralIn) {
            coralIn = true
            topMotor.stopMotor()
          }
        }

        if (rightSensorDetected()) {
          if (!unverticaling) {
            topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
            if (middleSensorDetected()) {
              // move left and slow down to prevent overshoot
              setMotorsLeft(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE / IntakeConstants.SIDES_SLOWDOWN_CONSTANT)
            } else {
              // move left
              setMotorsLeft()
            }
          } else { // unverticaling

            if (onlyRightSensor()) {
              // if it's only the right sensor we've successfully gotten the coral horizontal
              // so end the process
              unverticaling = false
            } else {
              // coral should be diagonal right now in between middle and right sensor
              // move right until just right sensor, pulling in slightly for steadiness
              topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
              setMotorsRight()
            }
          }
        }

        if (leftSensorDetected()) {
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          if (middleSensorDetected()) {
            // move right and slow down to prevent overshoot
            setMotorsRight(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE / IntakeConstants.SIDES_SLOWDOWN_CONSTANT)
          } else {
            // move right
            setMotorsRight()
          }
        }


        if (onlyMiddleSensor()) {
          if (!backSensorDetected()) {
            if (unverticaling) {
              // move right and out slowly
              setMotorsRight()
              topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / 3)
            } else {
              // run inwards till we hit back sensor
              setMotorsInwards()
            }
          } else { // hitting back sensor
            unverticaling = true
            // move right and out slowly
            setMotorsRight()
            topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / 3)
          }
        }
      },
      { },
      { coralIsHorizontal() }
    ).andThen(changePieceToCoral()).andThen(runOnce { topMotor.setVoltage(IntakeConstants.TOP_L1_HOLD) })
  }

  fun intakeToVertical(): Command {
    return FunctionalCommand(
      { },
      {
        if (leftSensorDetected() && rightSensorDetected()) {
          // if it's horizontal, just run it right
          //run in a bit because our side motors tweaking lowk
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          setMotorsRight()
        } else {
          // inwards
          setMotorsInwards()
        }
      },
      { },
      { backSensorDetected() },
    ).andThen(changePieceToCoral())
  }

  fun intakeAlgae(): Command {
    return Commands.sequence(
      setVoltageTop(IntakeConstants.ALGAE_INTAKE_VOLTAGE),
      changePieceToAlgae(),
    )
  }

  fun holdAlgae(): Command {
    return setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE)
  }

  fun outtakeL1(): Command {
    return Commands.sequence(
      setVoltageTop(IntakeConstants.TOP_L1_OUTTAKE),
      changePieceToNone(),
    )
  }

  fun outtakeCoral(): Command {
    return Commands.sequence(
      outwards(),
      changePieceToNone(),
    )
  }

  fun outtakeCoralPivot(): Command {
    return Commands.sequence(
      runOnce { setMotorsOutwards(-1.0) },
      changePieceToNone()
    )
  }

  fun outtakeAlgae(): Command {
    return Commands.sequence(
      setVoltageTop(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE),
      changePieceToNone(false),
    )
  }

  private fun laserCanDetected(laserCan: LaserCanInterface): Boolean {
    return laserCan.measurement != null && (
      laserCan.measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
        laserCan.measurement.distance_mm <= IntakeConstants.CORAL_DETECTION_THRESHOLD
      )
  }

  private fun laserCanIsPlugged(laserCan: LaserCanInterface): Boolean {
    return laserCan.measurement != null
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
      topMotor.statorCurrent.valueAsDouble >
        IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD
    }.onlyIf { RobotBase.isReal() }
      .andThen(WaitCommand(IntakeConstants.WAIT_BEFORE_ALGAE_IN))
      .andThen(runOnce { gamePiece = Piece.ALGAE })
      .andThen(holdAlgae())
  }

  private fun changePieceToCoral(): Command {
    return runOnce { gamePiece = Piece.CORAL }.andThen(stopMotors())
  }

  private fun changePieceToNone(coralOuttaken: Boolean = true): Command {
    return Commands.sequence(
      Commands.race(
        ConditionalCommand(
          // coral
          WaitUntilCommand { !coralDetected() },
          // algae
          WaitUntilCommand {
            topMotor.statorCurrent.valueAsDouble >
              IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD
          }.andThen(WaitCommand(IntakeConstants.WAIT_BEFORE_ALGAE_OUT))

        ) { coralOuttaken },
        WaitCommand(2.0)
      ),
      runOnce { gamePiece = Piece.NONE },
      stopMotors()
    )
  }

  fun resetPiece(): Command {
    return runOnce {
      gamePiece = Piece.CORAL
    }
  }

  fun stopMotors(): Command {
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

    DogLog.log("Intake/LaserCan/All Sensors Configured", allSensorsConfigured)
    DogLog.log("Intake/LaserCan/ Configured list", lasercanConfigured.toBooleanArray())

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
