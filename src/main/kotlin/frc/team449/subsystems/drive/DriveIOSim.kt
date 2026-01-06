package frc.team449.subsystems.drive

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Notifier
import frc.team449.Constants
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import java.util.function.Consumer

class DriveIOSim private constructor(
  driveConstants: SwerveDrivetrainConstants,
  moduleConstants: Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
) : DriveIOHardware(
  driveConstants,
  moduleConstants,
) {

  constructor(
    moduleConstants: Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>,
    driveConstants: SwerveDrivetrainConstants
  ) : this(
    driveConstants,
    sanitizeConstantsForSim(moduleConstants)
  )

  private val simulationConfig = DriveTrainSimulationConfig.Default()
    .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
    .withCustomModuleTranslations(moduleLocations)
    .withGyro(COTS.ofPigeon2())
    .withSwerveModule(
      SwerveModuleSimulationConfig(
        DCMotor.getKrakenX60(1), // Drive
        DCMotor.getNEO(1), // Steer
        moduleConstants[0].DriveMotorGearRatio,
        moduleConstants[0].SteerMotorGearRatio,
        Volts.of(moduleConstants[0].DriveFrictionVoltage),
        Volts.of(moduleConstants[0].SteerFrictionVoltage), // friction
        Meters.of(moduleConstants[0].WheelRadius),
        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
        Constants.DriveConstants.WHEEL_FRICTION_COEFFICIENT // COF
      )
    )

  private val mapleSimDrive = SwerveDriveSimulation(
    simulationConfig,
    Pose2d(3.0, 3.0, Rotation2d())
  )

  private val simTelemetryConsumer: Consumer<SwerveDriveState> = Consumer { swerveDriveState: SwerveDriveState ->
    swerveDriveState.Pose = mapleSimDrive.simulatedDriveTrainPose
    telemetryConsumer.accept(swerveDriveState)
  }

  private val simNotifier = Notifier {
    SimulatedArena.getInstance().simulationPeriodic()

    this.pigeon2.simState.setRawYaw(mapleSimDrive.simulatedDriveTrainPose.rotation.measure)
    this.pigeon2.simState.setAngularVelocityZ(
      RadiansPerSecond.of(mapleSimDrive.driveTrainSimulatedChassisSpeedsRobotRelative.omegaRadiansPerSecond)
    )
  }

  init {
    for (i in 0 until 4) {
      val realModule = this.getModule(i)
      val simModule = mapleSimDrive.modules[i]

      simModule.useDriveMotorController(
        SimulatedMotorController { _, _, _, _ ->
          realModule.driveMotor.simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage())

          realModule.driveMotor.simState.setRawRotorPosition(simModule.driveEncoderUnGearedPosition)
          realModule.driveMotor.simState.setRotorVelocity(simModule.driveEncoderUnGearedSpeed)

          realModule.driveMotor.simState.motorVoltageMeasure
        }
      )

      simModule.useSteerMotorController(
        SimulatedMotorController { _, _, _, _ ->
          realModule.steerMotor.simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage())

          realModule.steerMotor.simState.setRawRotorPosition(simModule.steerRelativeEncoderPosition)
          realModule.steerMotor.simState.setRotorVelocity(simModule.steerRelativeEncoderVelocity)

          realModule.encoder.simState.setRawPosition(simModule.steerAbsoluteFacing.measure)
          realModule.encoder.simState.setVelocity(simModule.steerAbsoluteEncoderSpeed)

          realModule.steerMotor.simState.motorVoltageMeasure
        }
      )
    }

    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive)
    registerTelemetry(simTelemetryConsumer)
    SimulatedArena.overrideSimulationTimings(Seconds.of(.005), 1)
    simNotifier.startPeriodic(.005)
  }

  override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
    super.updateInputs(inputs)
  }

  override fun resetOdometry(pose: Pose2d) {
    mapleSimDrive.setSimulationWorldPose(pose)
    super.resetOdometry(pose)
  }

  companion object {
    private fun sanitizeConstantsForSim(
      originalConstants: Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
    ): Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> {
      // create a new array to hold the modified constants
      return originalConstants.map { module ->
        // create a modified copy of the module constant
        module
          .withEncoderOffset(0.0)
          .withDriveMotorInverted(false)
          .withSteerMotorInverted(false)
          .withEncoderInverted(false)
          .withSteerMotorGains(
            module.SteerMotorGains
              .withKP(15.0)
              .withKD(0.5)
          )
          .withDriveFrictionVoltage(Volts.of(0.1))
          .withSteerFrictionVoltage(Volts.of(0.15))
      }.toTypedArray()
    }
  }
}
