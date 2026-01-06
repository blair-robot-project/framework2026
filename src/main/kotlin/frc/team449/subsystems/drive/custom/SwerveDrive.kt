package frc.team449.subsystems.drive.custom

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.Constants
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.gyro.GyroIO
import frc.team449.subsystems.drive.gyro.GyroIOInputsAutoLogged
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import java.util.function.Consumer
import kotlin.math.hypot
import kotlin.math.max

class SwerveDrive(
  private val gyroIO: GyroIO,
  flModuleIO: SwerveModuleIO,
  frModuleIO: SwerveModuleIO,
  blModuleIO: SwerveModuleIO,
  brModuleIO: SwerveModuleIO,
  val resetSimulationPoseCallBack: Consumer<Pose2d>
) : SubsystemBase() {
  private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()
  private var rawGyroRotation = Rotation2d()
  private val gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError)

  private val modules: Array<SwerveModule> = arrayOf(
    SwerveModule(flModuleIO, 0, TunerConstants.FrontLeft),
    SwerveModule(frModuleIO, 1, TunerConstants.FrontRight),
    SwerveModule(blModuleIO, 2, TunerConstants.BackLeft),
    SwerveModule(brModuleIO, 3, TunerConstants.BackRight)
  ) // FL, FR, BL, BR

  private val kinematics = SwerveDriveKinematics(*moduleTranslations)
  private val lastModulePositions: Array<SwerveModulePosition> = arrayOf(
    SwerveModulePosition(),
    SwerveModulePosition(),
    SwerveModulePosition(),
    SwerveModulePosition()
  )
  private val poseEstimator = SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d())

  private val sysId: SysIdRoutine = SysIdRoutine(
    SysIdRoutine.Config(null, null, null) { state: SysIdRoutineLog.State ->
      Logger.recordOutput(
        "Drive/SysIdState",
        state.toString()
      )
    },
    SysIdRoutine.Mechanism(
      { voltage: Voltage -> runCharacterization(voltage.`in`(Units.Volts)) },
      null,
      this
    )
  )

  init {
    // start odometry thread
    PhoenixOdometryThread.start()
  }

  override fun periodic() {
    odometryLock.lock() // prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs)
    Logger.processInputs("Drive/Gyro", gyroInputs)

    for (module in modules) {
      module.periodic()
    }

    odometryLock.unlock()

    // stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (module in modules) {
        module.stop()
      }
    }

    // log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
      Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
    }

    // Update odometry
    val sampleTimestamps: DoubleArray =
      modules[0].odometryTimestamps // All signals are sampled together
    val sampleCount = sampleTimestamps.size
    for (i in 0 until sampleCount) {
      // read wheel positions and deltas from each module
      val modulePositions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }
      val moduleDeltas = Array(4) { SwerveModulePosition() }
      for (moduleIndex in 0..3) {
        modulePositions[moduleIndex] = modules[moduleIndex].odometryPositions[i]
        moduleDeltas[moduleIndex] =
          SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters -
              lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle
          )
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i]
      } else {
        // use the angle delta from the kinematics and module deltas
        val twist = kinematics.toTwist2d(*moduleDeltas)
        rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
      }

      // apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions)
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE !== Constants.Mode.SIM)
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  fun runVelocity(speeds: ChassisSpeeds) {
    // calculate module setpoints
    val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
    val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts)

    // log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

    // send setpoints to modules
    for (i in 0..3) {
      modules[i].runSetpoint(setpointStates[i])
    }

    // log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
  }

  /** Runs the drive in a straight line with the specified drive output. */
  fun runCharacterization(output: Double) {
    for (i in 0..3) {
      modules[i].runCharacterization(output)
    }
  }

  /** Stops the drive. */
  fun stop() {
    runVelocity(ChassisSpeeds())
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  fun stopWithX() {
    val headings: Array<Rotation2d> = Array(4) { moduleTranslations[it].angle }
    kinematics.resetHeadings(*headings)
    stop()
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
    return this.run { runCharacterization(0.0) }
      .withTimeout(1.0)
      .andThen(sysId.quasistatic(direction))
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
    return this.run { runCharacterization(0.0) }
      .withTimeout(1.0)
      .andThen(sysId.dynamic(direction))
  }

  @get:AutoLogOutput(key = "SwerveStates/Measured")
  private val moduleStates: Array<SwerveModuleState>
    /** Returns the module states (turn angles and drive velocities) for all the modules.  */
    get() = Array(4) { modules[it].state }

  private val modulePositions: Array<SwerveModulePosition>
    /** Returns the module positions (turn angles and drive positions) for all the modules.  */
    get() = Array(4) { modules[it].position }

  @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private val chassisSpeeds: ChassisSpeeds
    /** Returns the measured chassis speeds of the robot.  */
    get() = kinematics.toChassisSpeeds(*moduleStates)

  val wheelRadiusCharacterizationPositions: DoubleArray
    /** Returns the position of each module in radians.  */
    get() = DoubleArray(4) { modules[it].wheelRadiusCharacterizationPosition }

  val fFCharacterizationVelocity: Double
    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    get() = modules.map { it.fFCharacterizationVelocity }.average()

  @get:AutoLogOutput(key = "Odometry/Robot")
  var pose: Pose2d
    /** Returns the current odometry pose. */
    get() = poseEstimator.estimatedPosition

    /** Resets the current odometry pose. */
    set(pose) {
      resetSimulationPoseCallBack.accept(pose)
      poseEstimator.resetPosition(rawGyroRotation, this.modulePositions, pose)
    }

  val rotation: Rotation2d
    /** Returns the current odometry rotation. */
    get() = this.pose.rotation

  /** Adds a new timestamped vision measurement.  */
  fun addVisionMeasurement(
    visionRobotPoseMeters: Pose2d,
    timestampSeconds: Double,
    visionMeasurementStdDevs: Matrix<N3, N1>
  ) {
    poseEstimator.addVisionMeasurement(
      visionRobotPoseMeters,
      timestampSeconds,
      visionMeasurementStdDevs
    )
  }

  val maxLinearSpeedMetersPerSec: Double
    /** Returns the maximum linear speed in meters per sec. */
    get() = TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond)

  val maxAngularSpeedRadPerSec: Double
    /** Returns the maximum angular speed in radians per sec. */
    get() = this.maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

  companion object {
    // TunerConstants doesn't include these constants, so they are declared locally
    const val ODOMETRY_FREQUENCY: Double = 100.0 // no CANivore :-(
    val DRIVE_BASE_RADIUS: Double = max(
      max(
        hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
      ),
      max(
        hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      )
    )

    // auto constants?!
    // TODO: adjust robot constants
    private const val ROBOT_MASS_KG = 74.088
    private const val ROBOT_MOI = 6.883
    private const val WHEEL_COF = 1.2

    val odometryLock: Lock = ReentrantLock()
    val moduleTranslations: Array<Translation2d> = arrayOf(
      Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    )

    // maple sim config
    val driveTrainSimulationConfig: DriveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
      .withRobotMass(Units.Kilograms.of(Constants.ROBOT_MASS_KG))
      .withCustomModuleTranslations(moduleTranslations)
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(
        SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60(1),
          DCMotor.getNEO(1),
          TunerConstants.FrontLeft.DriveMotorGearRatio, // use front left config for all
          TunerConstants.FrontLeft.SteerMotorGearRatio,
          Units.Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
          Units.Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
          Units.Meters.of(SwerveConstants.WHEEL_RADIUS_METERS),
          Units.KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
          SwerveConstants.WHEEL_COF
        )
      )
  }
}
