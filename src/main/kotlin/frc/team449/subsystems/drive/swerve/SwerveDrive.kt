package frc.team449.subsystems.drive.swerve

import choreo.trajectory.SwerveSample
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.auto.AutoConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken.Companion.createKrakenModule
import frc.team449.subsystems.drive.swerve.SwerveModuleNEO.Companion.createNEOModule

/**
 * A Swerve Drive chassis.
 * @param modules An array of [SwerveModule]s that are on the drivetrain.
 * @param maxLinearSpeed The maximum translation speed of the chassis.
 * @param maxRotSpeed The maximum rotation speed of the chassis.
 * @param field The SmartDashboard [Field2d] widget that shows the robot's pose.
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
  @Logged
  var maxLinearSpeed: Double,
  @Logged
  var accel: Double,
  @Logged
  var maxRotSpeed: Double,
  protected val field: Field2d,
  @Logged
  val maxModuleSpeed: Double
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
  var currentSpeeds = ChassisSpeeds()

  @Logged
  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  @Logged
  var desiredAngle = 0.0

  @Logged
  var desiredOmega = 0.0

  // Removed magic numbers
  @get:Logged
  val xController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0)

  @get:Logged
  val yController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0)

  @get:Logged
  val headingController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0)

  init {
    headingController.enableContinuousInput(-Math.PI, Math.PI)
  }
  fun followTrajectory(
    robot: Robot,
    sample: SwerveSample
  ) {
    desiredAngle = MathUtil.angleModulus(sample.heading)
    desiredOmega = sample.omega
    val speeds = ChassisSpeeds(
      sample.vx + xController.calculate(robot.poseSubsystem.pose.x, sample.x),
      sample.vy + yController.calculate(robot.poseSubsystem.pose.y, sample.y),
      sample.omega + headingController.calculate(
        robot.poseSubsystem.pose.rotation.minus(Rotation2d.fromRadians(MathUtil.angleModulus(sample.heading))).radians
      )
    )
    val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      robot.poseSubsystem.heading
    )

    // Apply the generated speeds
    set(newSpeeds)
  }

  fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
    // Converts the desired [ChassisSpeeds] into an array of [SwerveModuleState].
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)
    // Scale down module speed if a module is going faster than the max speed, and prevent early desaturation.
//    normalizeDrive(desiredModuleStates, desiredSpeeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      maxModuleSpeed
    )
    frontLeftModule.state = desiredModuleStates[0]
    frontRightModule.state = desiredModuleStates[1]
    backLeftModule.state = desiredModuleStates[2]
    backRightModule.state = desiredModuleStates[3]

    frontLeftModule.update()
    frontRightModule.update()
    backLeftModule.update()
    backRightModule.update()
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

  /** Stops the robot's drive. */
  fun driveStop(): Command {
    return runOnce {
      set(ChassisSpeeds(0.0, 0.0, 0.0))
    }
  }

  override fun periodic() {
    // Updates the robot's currentSpeeds.
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
    /** Create a [SwerveDrive] using [SwerveConstants]. */
    fun createSwerveKraken(field: Field2d): SwerveDrive {
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
          SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH / 2
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
          SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH / 2
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
          -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH / 2
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
          -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH / 2
        )
      )
      return if (isReal()) {
        SwerveDrive(
          frontLeftModule,
          frontRightModule,
          backLeftModule,
          backRightModule,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      } else {
        SwerveSim(
          frontLeftModule,
          frontRightModule,
          backLeftModule,
          backRightModule,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      }
    }

    fun createSwerveNEO(field: Field2d): SwerveDrive {
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
          SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH / 2
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
          SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH / 2
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
          -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH / 2
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
          -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH / 2
        )
      )
      return if (isReal()) {
        SwerveDrive(
          frontLeftModule,
          frontRightModule,
          backLeftModule,
          backRightModule,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      } else {
        SwerveSim(
          frontLeftModule,
          frontRightModule,
          backLeftModule,
          backRightModule,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      }
    }
  }
}
