package frc.team449.input

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import frc.team449.config.RobotConstants
import frc.team449.config.SwerveConstants
import frc.team449.util.Clock
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

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
class HolonomicOI(
  rotationRateLimit: AngularAcceleration,
  var maxLinearSpeed: LinearVelocity,
  var maxRotationalSpeed: AngularVelocity,
  var maxAccel: LinearAcceleration
) {

  private val rotRamp = SlewRateLimiter(rotationRateLimit.`in`(RadiansPerSecondPerSecond))
  private var xVelocity = Meters.per(Seconds).mutable(0.0)
  private var yVelocity = Meters.per(Seconds).mutable(0.0)
  private var rotationVelocity = Radians.per(Seconds).mutable(0.0)

  /**
   *
   * @param prevChassisCommand The previously calculated chassis output command
   * @param xThrottle The scalar Y axis of the strafing joystick
   * @param yThrottle The scalar X axis of the strafing joystick
   * @param rotThrottle The scalar X axis of the rotating joystick
   *
   * @return The new [ChassisSpeeds] for the given x, y and
   * rotation input from the joystick */
  fun calculate(prevChassisCommand: ChassisSpeeds, xThrottle: Double, yThrottle: Double, rotThrottle: Double): ChassisSpeeds {
    val dt = Clock.deltaTime.`in`(Seconds)

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
    val xScaled = ctrlRadius * cos(ctrlTheta) * maxLinearSpeed.`in`(MetersPerSecond)
    val yScaled = ctrlRadius * sin(ctrlTheta) * maxLinearSpeed.`in`(MetersPerSecond)
    val rotScaled = rotRamp.calculate(
      min(
        MathUtil.applyDeadband(
          abs(rotThrottle).pow(SwerveConstants.ROT_FILTER_ORDER),
          RobotConstants.ROTATION_DEADBAND,
          1.0,
        ),
        1.0,
      ) * -sign(rotThrottle) * maxRotationalSpeed.`in`(RadiansPerSecond),
    )

    if (RobotConstants.USE_ACCEL_LIMIT) {
      // Calculate and clamp the desired acceleration
      val dx = xScaled - prevChassisCommand.vxMetersPerSecond
      val dy = yScaled - prevChassisCommand.vyMetersPerSecond
      val accelerationMagnitude = hypot(dx / dt, dy / dt)
      val magAccClamped = MathUtil.clamp(
        accelerationMagnitude,
        -this.maxAccel.`in`(MetersPerSecondPerSecond),
        this.maxAccel.`in`(MetersPerSecondPerSecond)
      )

      // Scale the change in x and y the same way the acceleration would scale
      val factor = if (accelerationMagnitude == 0.0) 0.0 else magAccClamped / accelerationMagnitude
      val dxClamped = dx * factor
      val dyClamped = dy * factor

      xVelocity.mut_replace(MetersPerSecond.of(prevChassisCommand.vxMetersPerSecond + dxClamped))
      yVelocity.mut_replace(MetersPerSecond.of(prevChassisCommand.vyMetersPerSecond + dyClamped))
    } else {
      xVelocity.mut_replace(MetersPerSecond.of(xScaled))
      yVelocity.mut_replace(MetersPerSecond.of(yScaled))
    }
    rotationVelocity.mut_replace(RadiansPerSecond.of(rotScaled))

    return ChassisSpeeds(
      xVelocity,
      yVelocity,
      rotationVelocity
    )
    // TODO: field relative should be controlled on the drive end, not the input end.
  }
}
