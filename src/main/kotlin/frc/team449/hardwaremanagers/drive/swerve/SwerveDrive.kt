package frc.team449.hardwaremanagers.drive.swerve

import choreo.trajectory.SwerveSample
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.config.RobotConstants
import frc.team449.config.SwerveConstants
import frc.team449.hardware.gearbox.SwerveModule
import frc.team449.hardware.gearbox.SwerveModuleKraken.Companion.createKrakenModule
import frc.team449.hardware.gearbox.SwerveModuleNEO.Companion.createNEOModule
import frc.team449.hardware.gearbox.SwerveModuleSim.Companion.createModuleSim
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig

/**
 * A Swerve Drive chassis.
 * @param modules An array of [frc.team449.hardware.gearbox.SwerveModule]s that are on the drivetrain.
 */
open class SwerveDrive(
  @Logged
  val frontLeftModule: SwerveModule,
  @Logged
  val frontRightModule: SwerveModule,
  @Logged
  val backLeftModule: SwerveModule,
  @Logged
  val backRightModule: SwerveModule,
  val maxModuleSpeed: LinearVelocity
) : SubsystemBase() {

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  // This can't be logged because it doesn't have a struct, and marking it @NotLogged still tries to generate the broken struct call, so
  val kinematics = SwerveDriveKinematics(
    frontLeftModule.location,
    frontRightModule.location,
    backLeftModule.location,
    backRightModule.location,
  )

  /** The current speed of the robot's drive. */
  @Logged
  var currentSpeeds: ChassisSpeeds = ChassisSpeeds()

  @Logged
  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
    // Converts the desired [ChassisSpeeds] into an array of [SwerveModuleState].
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    // Scale down module speed if a module is going faster than the max speed, and prevent early desaturation.
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      maxModuleSpeed
    )

    // take scaled speeds and set module targets
    frontLeftModule.state = desiredModuleStates[0]
    frontRightModule.state = desiredModuleStates[1]
    backLeftModule.state = desiredModuleStates[2]
    backRightModule.state = desiredModuleStates[3]
  }

  fun followTrajectory(swerveSample: SwerveSample) {
    // todo, hmmmmm I don't like the idea of throwing an entire trajectory controller in here since this class should exclusively be for controlling the gearboxes
  }

  fun setVoltage(volts: Double) {
    frontLeftModule.setVoltage(volts)
    frontRightModule.setVoltage(volts)
    backLeftModule.setVoltage(volts)
    backRightModule.setVoltage(volts)
  }

  fun getModuleVel(): Double {
    return arrayOf(
      frontLeftModule.state.speedMetersPerSecond,
      frontRightModule.state.speedMetersPerSecond,
      backLeftModule.state.speedMetersPerSecond,
      backRightModule.state.speedMetersPerSecond
    ).average()
  }

  override fun periodic() {
    // drive to module targets
    frontLeftModule.update()
    frontRightModule.update()
    backLeftModule.update()
    backRightModule.update()

    // Updates the chassis's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      frontLeftModule.state,
      frontRightModule.state,
      backLeftModule.state,
      backRightModule.state
    )
  }

  /** Stops the robot's drive. */
  fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  /** @return An array of [SwerveModulePosition] for each module, containing distance and angle. */
  fun getPositions(): Array<SwerveModulePosition> {
    return arrayOf(
      frontLeftModule.position,
      frontRightModule.position,
      backLeftModule.position,
      backRightModule.position
    )
  }

  /** @return An array of [SwerveModuleState] for each module, containing speed and angle. */
  private fun getStates(): Array<SwerveModuleState> {
    return arrayOf(
      frontLeftModule.state,
      frontRightModule.state,
      backLeftModule.state,
      backRightModule.state
    )
  }

  companion object {
    /** Create a [SwerveDrive] using [frc.team449.config.SwerveConstants]. */
    fun createSwerveKraken(): SwerveDrive {
      // Real Modules
      val frontLeftModule = createKrakenModule(
        "FLModule",
        SwerveConstants.DRIVE_MOTOR_FL,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_FL,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_FL,
        SwerveConstants.TURN_ENC_OFFSET_FL,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val frontRightModule = createKrakenModule(
        "FRModule",
        SwerveConstants.DRIVE_MOTOR_FR,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_FR,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_FR,
        SwerveConstants.TURN_ENC_OFFSET_FR,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val backLeftModule = createKrakenModule(
        "BLModule",
        SwerveConstants.DRIVE_MOTOR_BL,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_BL,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_BL,
        SwerveConstants.TURN_ENC_OFFSET_BL,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          -SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val backRightModule = createKrakenModule(
        "BLModule",
        SwerveConstants.DRIVE_MOTOR_BR,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_BR,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_BR,
        SwerveConstants.TURN_ENC_OFFSET_BR,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          -SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      return SwerveDrive(
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule,
        SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
      )
    }

    fun createSwerveNEO(): SwerveDrive {
      val frontLeftModule = createNEOModule(
        "FLModule",
        SwerveConstants.DRIVE_MOTOR_FL,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_FL,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_FL,
        SwerveConstants.TURN_ENC_OFFSET_FL,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val frontRightModule = createNEOModule(
        "FRModule",
        SwerveConstants.DRIVE_MOTOR_FR,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_FR,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_FR,
        SwerveConstants.TURN_ENC_OFFSET_FR,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val backLeftModule = createNEOModule(
        "BLModule",
        SwerveConstants.DRIVE_MOTOR_BL,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_BL,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_BL,
        SwerveConstants.TURN_ENC_OFFSET_BL,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          -SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val backRightModule = createNEOModule(
        "BLModule",
        SwerveConstants.DRIVE_MOTOR_BR,
        SwerveConstants.DRIVE_INVERTED,
        SwerveConstants.TURN_MOTOR_BR,
        SwerveConstants.TURN_INVERTED,
        SwerveConstants.TURN_ENC_CHAN_BR,
        SwerveConstants.TURN_ENC_OFFSET_BR,
        SwerveConstants.TURN_ENC_INVERTED,
        Translation2d(
          -SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      return SwerveDrive(
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule,
        SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
      )
    }

    fun createSwerveSim(): SwerveDrive {
      val driveSim: SwerveDriveSimulation = SwerveDriveSimulation(
        DriveTrainSimulationConfig.Default()
          .withTrackLengthTrackWidth(
            SwerveConstants.TRACKWIDTH,
            SwerveConstants.WHEELBASE
          ).withSwerveModule(RobotConstants.MODULE_SIMULATION),
        RobotConstants.INITIAL_POSE
      )
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSim)
      val frontLeftModule = createModuleSim(
        "FLModule",
        driveSim.modules[0],
        Translation2d(
          SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val frontRightModule = createModuleSim(
        "FRModule",
        driveSim.modules[1],
        Translation2d(
          SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val backLeftModule = createModuleSim(
        "BLModule",
        driveSim.modules[2],
        Translation2d(
          -SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      val backRightModule = createModuleSim(
        "BLModule",
        driveSim.modules[3],
        Translation2d(
          -SwerveConstants.WHEELBASE.`in`(Meters) / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH.`in`(Meters) / 2
        )
      )
      return SwerveSim(
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule,
        SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED,
        driveSim
      )
    }
  }
}
