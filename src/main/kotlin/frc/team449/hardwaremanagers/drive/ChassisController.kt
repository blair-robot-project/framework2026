package frc.team449.hardwaremanagers.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.config.RobotConstants
import frc.team449.config.SwerveConstants
import frc.team449.util.Clock
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
  val constraints: DriveDynamics
) : Command() {

  private val rotRamp = SlewRateLimiter(constraints.maxAngularSpeedRate.`in`(Units.RadiansPerSecondPerSecond))
  private var xVelocity = Units.Meters.per(Units.Seconds).mutable(0.0)
  private var yVelocity = Units.Meters.per(Units.Seconds).mutable(0.0)
  private var rotationVelocity = Units.Radians.per(Units.Seconds).mutable(0.0)
  private var prevChassisSpeeds: ChassisSpeeds = ChassisSpeeds()

  init {
    chassis.defaultCommand = this
  }

  fun driverInput(xThrottle: Double, yThrottle: Double, rotThrottle: Double) {
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
    val rotScaled = rotRamp.calculate(
      min(
        MathUtil.applyDeadband(
          abs(rotThrottle).pow(SwerveConstants.ROT_FILTER_ORDER),
          RobotConstants.ROTATION_DEADBAND,
          1.0,
        ),
        1.0,
      ) * -sign(rotThrottle) * maxRotationalSpeed.`in`(Units.RadiansPerSecond),
    )


  }

  /**
   *
   * @param prevChassisCommand The previously calculated chassis output command
   * @param xThrottle The scalar Y axis of the strafing joystick
   * @param yThrottle The scalar X axis of the strafing joystick
   * @param rotThrottle The scalar X axis of the rotating joystick
   *
   * @return The new [edu.wpi.first.math.kinematics.ChassisSpeeds] for the given x, y and
   * rotation input from the joystick */
  fun calculate(): ChassisSpeeds {
    val dt = Clock.deltaTime.`in`(Units.Seconds)

    if (RobotConstants.USE_ACCEL_LIMIT) {
      // Calculate and clamp the desired acceleration
      val dx = xScaled - prevChassisSpeeds.vxMetersPerSecond
      val dy = yScaled - prevChassisSpeeds.vyMetersPerSecond
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

      xVelocity.mut_replace(Units.MetersPerSecond.of(prevChassisSpeeds.vxMetersPerSecond + dxClamped))
      yVelocity.mut_replace(Units.MetersPerSecond.of(prevChassisSpeeds.vyMetersPerSecond + dyClamped))
    } else {
      xVelocity.mut_replace(Units.MetersPerSecond.of(xScaled))
      yVelocity.mut_replace(Units.MetersPerSecond.of(yScaled))
    }
    rotationVelocity.mut_replace(Units.RadiansPerSecond.of(rotScaled))

    return ChassisSpeeds(
      xVelocity,
      yVelocity,
      rotationVelocity
    )

  }

  override fun execute() {
    val newChassisSpeeds: ChassisSpeeds = oi.calculate(prevChassisSpeeds, controller.leftX, controller.leftY, controller.rightX)
    prevChassisSpeeds = newChassisSpeeds

    // hijack velocity (apply skew compensation)
    val skew: Rotation2d = Rotation2d(newChassisSpeeds.omegaRadiansPerSecond * Clock.deltaTime.`in`(Seconds) * SwerveConstants.SKEW_CONSTANT)
    val skewedX = newChassisSpeeds.vxMetersPerSecond * skew.cos - newChassisSpeeds.vyMetersPerSecond * skew.sin
    val skewedY = newChassisSpeeds.vxMetersPerSecond * skew.sin + newChassisSpeeds.vyMetersPerSecond * skew.cos

    newChassisSpeeds.vxMetersPerSecond = skewedX
    newChassisSpeeds.vyMetersPerSecond = skewedY

    // hijack rotation
    if (headingLock) {
      if (checkSnapToAngelTolerance()) {
        exitSnapToAngle()
      } else {
        if (pointLock) {
          val fieldToRobot = poseEstimator.pose.translation
          val robotToPoint = orbitPoint - fieldToRobot
          rotCtrl.setpoint = robotToPoint.angle.radians
        }
        newChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
          rotCtrl.calculate(poseEstimator.heading.radians),
          -RobotConstants.ALIGN_ROT_SPEED,
          RobotConstants.ALIGN_ROT_SPEED,
        )
      }
    }

    // drive
    if (fieldRelative) {
      newChassisSpeeds.vxMetersPerSecond *= directionCompensation.invoke()
      newChassisSpeeds.vyMetersPerSecond *= directionCompensation.invoke()

      drive.set(ChassisSpeeds.fromFieldRelativeSpeeds(newChassisSpeeds, poseEstimator.heading))
    } else {
      drive.set(newChassisSpeeds)
    }
}

data class DriveDynamics(
  val maxSpeed: LinearVelocity,
  val maxAccel: LinearAcceleration,
  val maxAngularSpeed: AngularVelocity,
  val maxAngularSpeedRate: AngularAcceleration
)
