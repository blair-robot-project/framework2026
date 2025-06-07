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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.KrakenDogLog

class Intake(
  private val topMotor: TalonFX, // kraken x44
  private val rightMotor: TalonFX, // kraken x60
  private val leftMotor: TalonFX,  // kraken x60
  private val backCoralSensor: LaserCanInterface,
  private val leftCoralSensor: LaserCanInterface,
  private val rightCoralSensor: LaserCanInterface,
  private val middleCoralSensor: LaserCanInterface
) : SubsystemBase() {

  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setControl(VoltageOut(voltage)) }
    }
  }

  fun setVoltageTop(voltage: Double) : Command {return setVoltage(topMotor, voltage = voltage)}
  fun setVoltageRight(voltage: Double): Command{ return setVoltage(rightMotor, voltage = voltage)}
  fun setVoltageLeft(voltage: Double): Command{ return setVoltage(leftMotor, voltage = voltage)}
  fun setVoltageSides(voltage: Double): Command{ return setVoltage(rightMotor, leftMotor, voltage = voltage)}


  fun intakeCoralHorizontal(): Command {
    return run {
      setVoltageTop(IntakeConstants.CORAL_INTAKE_VOLTAGE)
      setVoltageSides(IntakeConstants.CORAL_INTAKE_VOLTAGE_LOWER) // change to some lower voltage
    }.until { coralHorizontalDetected() }
      .andThen(holdCoral())
  }

  fun intakeCoralVertical(): Command {
    return run {
      setVoltageSides(IntakeConstants.CORAL_INTAKE_VOLTAGE)
      setVoltageTop(IntakeConstants.CORAL_INTAKE_VOLTAGE_LOWER) // lower voltage
    }.until { coralVerticalDetected() }
      .andThen(holdCoral())
  }

  /*fun holdCoral(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position
    }
      .andThen(stop())
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }*/

  fun holdCoral(): Command{
    return runOnce{

      topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
      rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
      leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))

    }
      .andThen(stop()).andThen(run{
        topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble))
        leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble))
        rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble))
      })
  }


  fun holdCoralPivot():Command{
    return runOnce{

    }
  }

  fun holdCoralForward(): Command{
    return runOnce{
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
    return runOnce{
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
    val measurement: LaserCanInterface.Measurement = laserCan.measurement
    return (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= IntakeConstants.LASER_CAN_SENSOR_MIN_DISTANCE_MM)
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

    DogLog.log("Intake/LaserCan/Back Sensor Distance (mm)", backCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Left Sensor Distance (mm)", leftCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Right Sensor Distance (mm)", rightCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/LaserCan/Middle Sensor Distance (mm)", middleCoralSensor.measurement.distance_mm.toDouble())
    DogLog.log("Intake/ Back sensor",laserCanDetected(backCoralSensor) )
    DogLog.log("Intake/ Right sensor",laserCanDetected(rightCoralSensor) )
    DogLog.log("Intake/ Left sensor",laserCanDetected(leftCoralSensor) )
    DogLog.log("Intake/ Middle sensor",laserCanDetected(middleCoralSensor) )
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
        LaserCan(IntakeConstants.BACK_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.LEFT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.RIGHT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.MIDDLE_CORAL_SENSOR_CAN_ID)
      )
    }
  }
}
