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
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.Commands.*
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

  private val sensors = listOf<LaserCanInterface>(
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
      try{
      sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
      sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
      sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
      } catch (_:Exception){
        sensorInitFailed = true
        failedSensors.indexOf(sensors)
      }
    }
  }

  private fun setVoltageTop(voltage: Double): Command { return setVoltage(topMotor, voltage = voltage) }
  private fun setVoltageSides(voltage: Double): Command { return setVoltage(rightMotor, leftMotor, voltage = voltage) }

   fun toTheRight(): Command {
    return runOnce {
      rightMotor.setVoltage(-IntakeConstants.RUN_SIDES)
      leftMotor.setVoltage(IntakeConstants.RUN_SIDES)    }
  }

   fun toTheLeft(): Command {
    return runOnce {
      rightMotor.setVoltage(IntakeConstants.RUN_SIDES)
      leftMotor.setVoltage(-IntakeConstants.RUN_SIDES)
    }
  }

  private fun inwards(): Command {
    return runOnce {
      topMotor.setVoltage(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
      rightMotor.setVoltage(IntakeConstants.SIDE_ROLLER_IN_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.SIDE_ROLLER_IN_VOLTAGE)
    }
  }

  private fun topRollerIn(): Command{
    return runOnce{
      topMotor.setVoltage(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
    }
  }
  fun outwards(): Command {
    return run {
      topMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      rightMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      leftMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
    }
  }

  fun intakeCoralHorizontal(): Command {
    changePieceToCoral().schedule()
    return ConditionalCommand( // if coral not detected
      setVoltageTop(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
        .until({ coralDetected() }),
      ConditionalCommand( // if coral detected by the three sensors
        setVoltageSides(IntakeConstants.RUN_SIDES)
          .until { !rightLaserCanDetected() }
          .andThen(setVoltageSides(IntakeConstants.RUN_SIDES)).withTimeout(0.5)
          .andThen(holdCoral()),
        ConditionalCommand( // if coral detected by right sensor
          setVoltageSides(IntakeConstants.RUN_SIDES)
            .until { coralHorizontalDetected() },
          ConditionalCommand( // if coral detected by left sensor
            setVoltageSides(IntakeConstants.RUN_SIDES)
              .until { coralHorizontalDetected() },
            setVoltageTop(IntakeConstants.TOP_ROLLER_IN_VOLTAGE)
          ) { leftLaserCanDetected() }
        ) { rightLaserCanDetected() }
      ) { coralHorizontalDetected() }
    ) { coralNotDetected() }
  }


  fun horizontal(): Command{
    return sequence(
      topRollerIn().until{coralDetected()},
      WaitUntilCommand{coralHorizontalDetected()}.withDeadline(
        run {
          when{
            !leftLaserCanDetected() && rightLaserCanDetected()
              -> toTheRight().until{rightLaserCanDetected()}

            leftLaserCanDetected() && !rightLaserCanDetected()
            -> toTheLeft().until{leftLaserCanDetected()}

            !middleLaserCanDetected()
            -> topRollerIn()

            else -> waitSeconds(1.0)
          }.repeatedly()
        }
      ),
      holdCoral()
    )
  }




  fun intakeCoralVertical(): Command {
    changePieceToCoral().schedule()
    return Commands.sequence(
      inwards(),
      WaitUntilCommand { coralDetected() },
      ConditionalCommand( // if coral detected by middle sensor
        // its in a good position for vertical so just run it back and then hold
        Commands.sequence(
          inwards(),
          WaitUntilCommand { backLaserCanDetected() },
          holdCoral()
        ),
        // otherwise run it to a side until it can be intaked at an angle
        Commands.sequence(
          ConditionalCommand( // if coral detected by left sensor
            // run it to the right and then intake
            toTheRight(),
            // if it wasn't (detected by right) run it to the left
            toTheLeft()
          ) { leftLaserCanDetected() },
          // then intake it
          WaitUntilCommand { middleLaserCanDetected() },
          inwards(),
          WaitUntilCommand{ backLaserCanDetected() },
          holdCoral())
      ) { middleLaserCanDetected() }
    )
  }

  fun holdCoral(): Command {
    return runOnce {
      topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
      rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
      leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
    }
      .andThen(stop())
  }



  fun holdCoralToFront(): Command{
    return sequence(
      outwards().onlyWhile {backLaserCanDetected()},
      holdCoral(),
      WaitCommand(0.25),
      inwards().until{middleLaserCanDetected() && backLaserCanDetected()}.withTimeout(0.3),
      holdCoral()
    )
  }

  fun holdCoralToPivot(): Command{
    return sequence(
      inwards().onlyWhile{middleLaserCanDetected()},
      holdCoral(),
      WaitCommand(0.5),
      outwards().until{middleLaserCanDetected() && backLaserCanDetected()}.withTimeout(0.2),
      holdCoral()
      )
  }

  fun holdCoralForward(): Command {
    return stop().andThen(
      run {
        topMotor.setControl(PositionDutyCycle(topMotor.position.valueAsDouble + 1))
        rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble + 1))
        leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble + 1))
      }
    )
  }

  fun holdCoralForwardAuto(): Command {
    return stop().andThen(
      run {
        topMotor.setControl(PositionDutyCycle(topMotor.position.valueAsDouble + 1.875))
        rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble + 1.875))
        leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble + 1.875))
      }
    )
  }

  fun intakeAlgae(): Command {
    currentSenseAlgae().schedule()
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
    return setVoltageSides(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
  }

  fun outtakeCoralPivot(): Command {
    changePieceToNone(true).schedule()
    return setVoltageSides(IntakeConstants.CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE)
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

  fun coralVerticalDetected(): Boolean {
    return laserCanDetected(backCoralSensor) && laserCanDetected(middleCoralSensor) && !laserCanDetected(leftCoralSensor) && !laserCanDetected(rightCoralSensor)
  }

  fun coralHorizontalDetected(): Boolean {
    return !laserCanDetected(backCoralSensor) && laserCanDetected(leftCoralSensor) && laserCanDetected(rightCoralSensor) && laserCanDetected(middleCoralSensor)
  }

  private fun rightLaserCanDetected(): Boolean {
    return laserCanDetected(rightCoralSensor)
  }

  private fun leftLaserCanDetected(): Boolean {
    return laserCanDetected(leftCoralSensor)
  }

  private fun middleLaserCanDetected(): Boolean {
    return laserCanDetected(middleCoralSensor)
  }

  private fun backLaserCanDetected(): Boolean {
    return laserCanDetected(backCoralSensor)
  }

  // Coral is controlled by the Intake
  fun coralControlled(): Boolean {
    return coralVerticalDetected() || coralHorizontalDetected()
  }

  // Coral is not vertical or horizontal but is detected by one of the sensors
  fun coralMisplaced(): Boolean {
    return coralDetected() && !coralVerticalDetected() && !coralHorizontalDetected()
  }

  fun algaeDetected(): Boolean {
    return gamePiece == Piece.ALGAE
  }

  private fun currentSenseAlgae(): Command {
    return WaitUntilCommand {
      topMotor.motorStallCurrent.valueAsDouble >
        IntakeConstants.ALGAE_STALL_TRESHHOLD
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
        WaitUntilCommand { !coralDetected() }.onlyIf { RobotBase.isReal() },
        WaitCommand(0.1).andThen(
          WaitUntilCommand {
            topMotor.motorStallCurrent.valueAsDouble <
              IntakeConstants.ALGAE_STALL_TRESHHOLD
          }.onlyIf { RobotBase.isReal() }
        )
      ) { coralOuttaken },
      WaitCommand(0.1),
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

    DogLog.log("Intake/HorizontalIntake", coralHorizontalDetected())
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
