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
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.select
import frc.team449.system.motor.KrakenDogLog

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
  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setControl(VoltageOut(voltage)) }
    }
  }

  init {
    for(sensor in sensors) {
      sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
      sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
      sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
    }
  }

  fun setVoltageTop(voltage: Double): Command { return setVoltage(topMotor, voltage = voltage) }
  fun setVoltageRight(voltage: Double): Command { return setVoltage(rightMotor, voltage = voltage) }
  fun setVoltageLeft(voltage: Double): Command { return setVoltage(leftMotor, voltage = voltage) }
  fun setVoltageSides(voltage: Double): Command { return setVoltage(rightMotor, leftMotor, voltage = voltage) }



  fun toTheRight(): Command{
    return runOnce{setVoltageSides(IntakeConstants.RUN_SIDES_TO_RIGHT)
    }
  }

  fun toTheLeft(): Command{
    return runOnce{setVoltageSides(IntakeConstants.RUN_SIDES_TO_LEFT)
  }
  }
  fun inwards(): Command{
    return run{
      setVoltageRight(IntakeConstants.RUN_SIDES_TO_RIGHT)
      setVoltageLeft(IntakeConstants.RUN_SIDES_TO_LEFT)
      setVoltageTop(IntakeConstants.CORAL_INTAKE_VOLTAGE)
    }
  }

  fun outwards(): Command{
    return run{
      setVoltageRight(IntakeConstants.RUN_SIDES_TO_RIGHT)
      setVoltageLeft(IntakeConstants.RUN_SIDES_TO_LEFT)
      setVoltageTop(IntakeConstants.CORAL_INTAKE_VOLTAGE)
    }
  }




  fun intakeCoralHorizontal(): Command {
    return ConditionalCommand(// if coral not detected
      setVoltageTop(IntakeConstants.TOP_ROLLER_VOLTAGE)
        .until({ coralDetected()}),
      ConditionalCommand(// if coral detected by the three sensors
        setVoltageSides(IntakeConstants.RUN_SIDES_TO_LEFT)
          .until { !rightlaserCan() }
          .andThen(setVoltageSides(IntakeConstants.RUN_SIDES_TO_RIGHT)).withTimeout(0.5)
          .andThen(holdCoral()),
        ConditionalCommand(// if coral detected by right sensor
          setVoltageSides(IntakeConstants.RUN_SIDES_TO_LEFT)
            .until { coralHorizontalDetected() },
          ConditionalCommand(//if coral detected by left sensor
            setVoltageSides(IntakeConstants.RUN_SIDES_TO_RIGHT)
              .until { coralHorizontalDetected() },
            setVoltageTop(IntakeConstants.TOP_ROLLER_VOLTAGE)
          ){leftLaserCan()}
        ){rightlaserCan()}
      ){coralHorizontalDetected()}
    ){coralNotDetected()}
  }





  fun intakeCoralVertical(): Command {
    return ConditionalCommand(//if coral not detected
      inwards().until({coralDetected()}),
      ConditionalCommand(//if coral detected by middle sensor
        inwards().until({backLaserCan()})
          .andThen(holdCoral()),
        ConditionalCommand(// if coral detected by left sensor
          toTheRight().until({middleLaserCan()})
            .andThen(inwards().until({ backLaserCan() })
              .andThen(holdCoral())),
          ConditionalCommand(// if coral detected by right sensor
            toTheLeft().until({middleLaserCan()})
              .andThen(inwards().until({ backLaserCan() })
                .andThen(holdCoral())),
            inwards()
          ){rightlaserCan()}
        ){leftLaserCan()}
      ){middleLaserCan()}
    ){coralNotDetected()}
  }


//
//  fun intakeAction(): Command{
//    return select(
//      mapOf(
//        IntakeState.FIND_CORAL to Commands.sequence(
//          setVoltageTop(IntakeConstants.TOP_ROLLER_VOLTAGE)
//          .until{coralDetected()}
//        ),
//
//        IntakeState.CORAL_IS_HORIZONTAL to Commands.sequence(
//          toTheLeft().withTimeout(0.5).andThen(holdCoral())
//        ),
//
//        IntakeState.CORAL_IS_RIGHT to Commands.sequence(
//          toTheLeft().until{coralHorizontalDetected()}
//            .andThen( toTheLeft().withTimeout(0.5).andThen(holdCoral()))
//        ),
//
//        IntakeState.CORAL_IS_LEFT to Commands.sequence(
//          toTheRight().until{coralHorizontalDetected()}
//            .andThen( toTheLeft().withTimeout(0.5).andThen(holdCoral()))
//        ),
//
//        IntakeState.CORAL_IS_MIDDLE to Commands.sequence(
//
//        )
//      )
//    ){updateCoralState()}
//  }
//
//  fun updateCoralState(): IntakeState {
//    var coralState = when{
//      coralNotDetected() -> IntakeState.FIND_CORAL
//      coralHorizontalDetected() -> IntakeState.CORAL_IS_HORIZONTAL
//      rightlaserCan() -> IntakeState.CORAL_IS_RIGHT
//      leftLaserCan() -> IntakeState.CORAL_IS_LEFT
//      coralVerticalDetected() -> IntakeState.CORAL_IS_VERTICAL
//      middleLaserCan() -> IntakeState.CORAL_IS_MIDDLE
//      else -> IntakeState.NONE
//    }
//    return coralState
//  }
//



  fun holdCoral(): Command {
    return runOnce {
      topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
      rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
      leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
    }
      .andThen(stop()).andThen(
        run {
          topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
          leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
          rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
        }
      )
  }

  fun holdCoralPivot(): Command {
    return runOnce {
    }
  }

  fun holdCoralForward(): Command {
    return runOnce {
      topMotor.setControl(PositionDutyCycle(topMotor.position.valueAsDouble))
      rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble))
      leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble))

      topMotor.setPosition(topMotor.position.valueAsDouble + 1)
      rightMotor.setPosition(rightMotor.position.valueAsDouble + 1)
      leftMotor.setPosition(leftMotor.position.valueAsDouble + 1)
    }
      .andThen(stop())
  }

  fun holdCoralForwardAuto(): Command {
    return runOnce {
      topMotor.setControl(PositionDutyCycle(topMotor.position.valueAsDouble))
      rightMotor.setControl(PositionDutyCycle(rightMotor.position.valueAsDouble))
      leftMotor.setControl(PositionDutyCycle(leftMotor.position.valueAsDouble))

      topMotor.setPosition(topMotor.position.valueAsDouble + 1.875)
      rightMotor.setPosition(rightMotor.position.valueAsDouble + 1.875)
      leftMotor.setPosition(leftMotor.position.valueAsDouble + 1.875)
    }
      .andThen(stop())
  }



  /*fun holdCoralForward(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position + 1.0
    }
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }

  fun holdCoralForwardAuto(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position + 1.875
    }
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }

  fun holdCoralBackwards(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position - 1.0
    }
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }*/

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
    val measurement: LaserCanInterface.Measurement? = laserCan.measurement
    return measurement != null && (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
      measurement.distance_mm <= IntakeConstants.CORAL_DETECTION_THRESHOLD)
 }

  private fun laserCanUnplugged(laserCan: LaserCanInterface): Boolean {
    val measurement = laserCan.measurement
      return measurement.status == null // returns TRUE if the laserCan is unplugged

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

  fun rightlaserCan():Boolean {
    return laserCanDetected(rightCoralSensor)
  }

  fun leftLaserCan(): Boolean {
    return laserCanDetected(leftCoralSensor)
  }

  fun middleLaserCan(): Boolean{
    return laserCanDetected(middleCoralSensor)
  }

  fun backLaserCan(): Boolean{
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

    KrakenDogLog.log("Intake/topMotor", topMotor)
    KrakenDogLog.log("Intake/rightMotor", rightMotor)
    KrakenDogLog.log("Intake/leftMotor", leftMotor)

    DogLog.log("Intake/HorizontalIntake", coralHorizontalDetected())

    DogLog.log("Intake/LaserCan/Back Sensor Distance (mm)", if(backCoralSensor.measurement == null) -1.0 else backCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Left Sensor Distance (mm)", if(leftCoralSensor.measurement== null) -1.0 else leftCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Right Sensor Distance (mm)", if (rightCoralSensor.measurement ==  null) -1.0 else rightCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Middle Sensor Distance (mm)", if (middleCoralSensor.measurement ==  null) -1.0 else middleCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/ Back sensor", laserCanDetected(backCoralSensor))
    DogLog.log("Intake/ Right sensor", laserCanDetected(rightCoralSensor))
    DogLog.log("Intake/ Left sensor", laserCanDetected(leftCoralSensor))
    DogLog.log("Intake/ Middle sensor", laserCanDetected(middleCoralSensor))

    DogLog.log("Intake/ Middle sensor state", laserCanUnplugged(middleCoralSensor))
    DogLog.log("Intake/ Right sensor state", laserCanUnplugged(rightCoralSensor))
    DogLog.log("Intake/ Left sensor state", laserCanUnplugged(leftCoralSensor))
    DogLog.log("Intake/ Back sensor state", laserCanUnplugged(backCoralSensor))

  }

  companion object {
    fun createIntake(): Intake {
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
        LaserCan(IntakeConstants.BACK_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.LEFT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.RIGHT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.MIDDLE_CORAL_SENSOR_CAN_ID)
      )
    }
  }
}


