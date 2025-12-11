package frc.team449.hardwaremanagers.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.team449.config.RobotConstants
import frc.team449.config.SwerveConstants
import frc.team449.hardwaremanagers.PoseSubsystem
import frc.team449.util.Clock
import java.util.function.DoubleSupplier
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

/**
 * Create an OI for controlling a holonomic drivetrain (probably swerve).
 * The x and y axes on one joystick are used to control x and y velocity (m/s),
 * while the x-axis on another joystick is used to control rotational velocity (m/s).
 * <p> The magnitude of the acceleration is clamped
 * <p>Note that the joystick's X
 * axis corresponds to the robot's/field's Y and vice versa
 *
 * @param rotRamp Used to ramp angular velocity
 * @param maxLinearSpeed Maximum desired linear drive speed (Unit agnostic)
 * @param maxRotationalSpeed Maximum desired angular rotation speed (Unit agnostic)
 * @param maxAccel Max desired drive acceleration (Unit agnostic), used for scaling speed
 * be relative to the field rather than relative to the robot. This better be true.
 */
class ChassisController(
  val chassis: SwerveChassis,
  var constraints: DriveDynamics,
  val poseEstimator: PoseSubsystem,
  private val xAxisSupplier: DoubleSupplier,
  private val yAxisSupplier: DoubleSupplier,
  private val rotationAxisSupplier: DoubleSupplier,
  private val autoXAxisPID: PIDController = PIDController(7.5, 0.0, 0.0), // TODO This needs to be pulling from config file by default
  private val autoYAxisPID: PIDController = PIDController(7.5, 0.0, 0.0),
  private val autoHeadingPID: PIDController = PIDController(5.0, 0.0, 0.0),
  private val tolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3.0))
) : Command() {

  private val rotRamp = SlewRateLimiter(constraints.maxAngularSpeedRate.`in`(Units.RadiansPerSecondPerSecond))

  private var prevChassisSpeeds: ChassisSpeeds = ChassisSpeeds()

  private val allianceCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) PI else 0.0 }
  private val directionCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) -1.0 else 1.0 }

  private var headingOverride = false
  private var pointLock = false
  private var orbitPoint: Translation2d = Translation2d()
  private var fieldRelative = true

  private val autopilotConstraints: APConstraints? = APConstraints()
    .withAcceleration(RobotConstants.MAX_ACCEL.`in`(MetersPerSecondPerSecond))
    .withJerk(2.0)

  private val autopilotProfile: APProfile? = APProfile(autopilotConstraints)
    .withErrorXY(tolerance.measureX)
    .withErrorTheta(tolerance.rotation.measure)
    .withBeelineRadius(Centimeters.of(8.0))

  val autopilotCalculator: Autopilot = Autopilot(autopilotProfile)

  init {
    addRequirements(chassis)
    chassis.defaultCommand = this

    autoHeadingPID.enableContinuousInput(-PI, PI)

    // Set tolerances from the given pose tolerance
    autoXAxisPID.setTolerance(tolerance.x)
    autoYAxisPID.setTolerance(tolerance.y)
    autoHeadingPID.setTolerance(tolerance.rotation.radians)

    autoXAxisPID.reset()
    autoYAxisPID.reset()
    autoHeadingPID.reset()
  }

  fun getDriverInput(): ChassisSpeeds {
    val xThrottle = xAxisSupplier.asDouble
    val yThrottle = yAxisSupplier.asDouble
    val rotThrottle = rotationAxisSupplier.asDouble

    // Polar conversion
    val ctrlRadius =
      MathUtil
        .applyDeadband(
          min(sqrt(xThrottle.pow(2) + yThrottle.pow(2)), 1.0),
          RobotConstants.DRIVE_RADIUS_DEADBAND,
          1.0,
        ).pow(SwerveConstants.JOYSTICK_FILTER_ORDER)
    val ctrlTheta = atan2(xThrottle, yThrottle)

    // Normalize axes and ease rotation (convert units to expected)
    val xScaled = ctrlRadius * cos(ctrlTheta) * constraints.maxSpeed.`in`(Units.MetersPerSecond)
    val yScaled = ctrlRadius * sin(ctrlTheta) * constraints.maxSpeed.`in`(Units.MetersPerSecond)
    val rotScaled = MathUtil.applyDeadband(
      abs(rotThrottle).pow(SwerveConstants.ROT_FILTER_ORDER),
      RobotConstants.ROTATION_DEADBAND,
      1.0,
    ) * -sign(rotThrottle) * constraints.maxAngularSpeed.`in`(Units.RadiansPerSecond)

    return ChassisSpeeds(xScaled, yScaled, rotScaled)
  }

  fun changeDriveDynamics(constraints: DriveDynamics): Command {
    return RunCommand({
      this.constraints = constraints
    })
  }

  fun snapToAngle(angle: Rotation2d): Command {
    return RunCommand({
      this.autoHeadingPID.setpoint = MathUtil.angleModulus(angle.radians + allianceCompensation.invoke())
      this.headingOverride = true
      this.pointLock = false
    })
  }

  fun snapToPoint(point: Translation2d): Command {
    return RunCommand({
      this.orbitPoint = point
      this.headingOverride = true
      this.pointLock = true
    })
  }

  fun constraintModifier(deltaTime: Time, prevChassisSpeeds: ChassisSpeeds, targetChassisSpeeds: ChassisSpeeds) {
    val dt = deltaTime.`in`(Seconds)

    // Apply acceleration scaling
    if (RobotConstants.USE_ACCEL_LIMIT) {
      // Calculate and clamp the desired acceleration
      val dx = targetChassisSpeeds.vxMetersPerSecond - prevChassisSpeeds.vxMetersPerSecond
      val dy = targetChassisSpeeds.vyMetersPerSecond - prevChassisSpeeds.vyMetersPerSecond
      val accelerationMagnitude = hypot(dx / dt, dy / dt)
      val magAccClamped = MathUtil.clamp(
        accelerationMagnitude,
        -constraints.maxAccel.`in`(Units.MetersPerSecondPerSecond),
        constraints.maxAccel.`in`(Units.MetersPerSecondPerSecond)
      )

      // Scale the change in x and y the same way the acceleration would scale
      val factor = if (accelerationMagnitude == 0.0) 0.0 else magAccClamped / accelerationMagnitude
      val dxClamped = dx * factor
      val dyClamped = dy * factor

      targetChassisSpeeds.vxMetersPerSecond = prevChassisSpeeds.vxMetersPerSecond + dxClamped
      targetChassisSpeeds.vyMetersPerSecond = prevChassisSpeeds.vyMetersPerSecond + dyClamped
    }

    // apply rotation scaling
    targetChassisSpeeds.omegaRadiansPerSecond = rotRamp.calculate(targetChassisSpeeds.omegaRadiansPerSecond)
  }
  fun rotationModifier(deltaTime: Time, targetChassisSpeeds: ChassisSpeeds) {
    if (pointLock) {
      val fieldToRobot = poseEstimator.pose.translation
      val robotToPoint = orbitPoint - fieldToRobot
      autoHeadingPID.setpoint = robotToPoint.angle.radians
    }
    targetChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
      autoHeadingPID.calculate(poseEstimator.heading.radians),
      -RobotConstants.ALIGN_ROT_SPEED,
      RobotConstants.ALIGN_ROT_SPEED,
    )
  }

  fun motionVectorModifier(deltaTime: Time, targetChassisSpeeds: ChassisSpeeds) {
    // apply skew compensation
    val skew = Rotation2d(targetChassisSpeeds.omegaRadiansPerSecond * Clock.deltaTime.`in`(Seconds) * SwerveConstants.SKEW_CONSTANT)
    val skewedX = targetChassisSpeeds.vxMetersPerSecond * skew.cos - targetChassisSpeeds.vyMetersPerSecond * skew.sin
    val skewedY = targetChassisSpeeds.vxMetersPerSecond * skew.sin + targetChassisSpeeds.vyMetersPerSecond * skew.cos

    targetChassisSpeeds.vxMetersPerSecond = skewedX
    targetChassisSpeeds.vyMetersPerSecond = skewedY
  }

  // Takes target, applies physical constraints, send to IK calculator/gearbox manager
  override fun execute() {
    val dt = Clock.deltaTime
    var targetChassisSpeeds: ChassisSpeeds

    targetChassisSpeeds = getDriverInput()
    rotationModifier(dt, targetChassisSpeeds)
    motionVectorModifier(dt, targetChassisSpeeds)
    constraintModifier(dt, prevChassisSpeeds, targetChassisSpeeds)

    // drive
    if (fieldRelative) {
      targetChassisSpeeds.vxMetersPerSecond *= directionCompensation.invoke()
      targetChassisSpeeds.vyMetersPerSecond *= directionCompensation.invoke()

      chassis.set(ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, poseEstimator.heading))
    } else {
      chassis.set(targetChassisSpeeds)
    }

    // update previous
    prevChassisSpeeds = targetChassisSpeeds
  }
}

data class DriveDynamics(
  val maxSpeed: LinearVelocity,
  val maxAccel: LinearAcceleration,
  val maxAngularSpeed: AngularVelocity,
  val maxAngularSpeedRate: AngularAcceleration
)
