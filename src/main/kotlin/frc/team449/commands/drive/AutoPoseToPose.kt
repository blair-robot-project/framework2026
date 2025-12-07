package frc.team449.commands.drive

import com.therekrab.autopilot.APConstraints
import com.therekrab.autopilot.APProfile
import com.therekrab.autopilot.Autopilot
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import frc.team449.config.RobotConstants
import kotlin.math.PI
import kotlin.math.hypot

object AutoPoseToPose {

  // TODO This needs to be configurable with constants
  private val xPID: PIDController = PIDController(7.5, 0.0, 0.0)
  private val yPID: PIDController = PIDController(7.5, 0.0, 0.0)
  private val headingPID: PIDController = PIDController(5.0, 0.0, 0.0)
  private val tolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3.0))

  private val autopilotConstraints: APConstraints? = APConstraints()
    .withAcceleration(RobotConstants.MAX_ACCEL.`in`(MetersPerSecondPerSecond))
    .withJerk(2.0)

  private val autopilotProfile: APProfile? = APProfile(autopilotConstraints)
    .withErrorXY(tolerance.measureX)
    .withErrorTheta(tolerance.rotation.measure)
    .withBeelineRadius(Centimeters.of(8.0))

  val autopilotCalculator: Autopilot = Autopilot(autopilotProfile)

  init {
    headingPID.enableContinuousInput(-PI, PI)

    // Set tolerances from the given pose tolerance
    xPID.setTolerance(tolerance.x)
    yPID.setTolerance(tolerance.y)
    headingPID.setTolerance(tolerance.rotation.radians)

    xPID.reset()
    yPID.reset()
    headingPID.reset()
  }

  fun calculate(currentPose: Pose2d, endPose: Pose2d): ChassisSpeeds {
    xPID.setpoint = endPose.x
    yPID.setpoint = endPose.y
    headingPID.setpoint = endPose.rotation.radians

    val speedClampMeters = RobotConstants.MAX_LINEAR_SPEED.`in`(MetersPerSecond)
    val rotationSpeedClampRad = RobotConstants.MAX_ROT_SPEED.`in`(RadiansPerSecond)

    // Calculate the feedback for X, Y, and theta using their respective controllers
    val xFeedback = MathUtil.clamp(xPID.calculate(currentPose.measureX.`in`(Meters)), -speedClampMeters, speedClampMeters)
    val yFeedback = MathUtil.clamp(yPID.calculate(currentPose.measureY.`in`(Meters)), -speedClampMeters, speedClampMeters)
    val headingFeedback = MathUtil.clamp(headingPID.calculate(currentPose.rotation.radians), -rotationSpeedClampRad, rotationSpeedClampRad)

    return ChassisSpeeds.fromFieldRelativeSpeeds(
      xFeedback,
      yFeedback,
      headingFeedback,
      currentPose.rotation
    )
  }

  fun calculate(currentPose: Pose2d, endPose: Pose2d, speedClamp: LinearVelocity, rotationSpeedClamp: AngularVelocity): ChassisSpeeds {
    xPID.setpoint = endPose.x
    yPID.setpoint = endPose.y
    headingPID.setpoint = endPose.rotation.radians

    val speedClampMeters = speedClamp.`in`(MetersPerSecond)
    val rotationSpeedClampRad = rotationSpeedClamp.`in`(RadiansPerSecond)

    // Calculate the feedback for X, Y, and theta using their respective controllers
    val xFeedback = MathUtil.clamp(xPID.calculate(currentPose.measureX.`in`(Meters)), -speedClampMeters, speedClampMeters)
    val yFeedback = MathUtil.clamp(yPID.calculate(currentPose.measureY.`in`(Meters)), -speedClampMeters, speedClampMeters)
    val headingFeedback = MathUtil.clamp(headingPID.calculate(currentPose.rotation.radians), -rotationSpeedClampRad, rotationSpeedClampRad)

    return ChassisSpeeds.fromFieldRelativeSpeeds(
      xFeedback,
      yFeedback,
      headingFeedback,
      currentPose.rotation
    )
  }

  fun isFinished(currentSpeeds: ChassisSpeeds): Boolean {
    return xPID.atSetpoint() && yPID.atSetpoint() && headingPID.atSetpoint() &&
      hypot(
        currentSpeeds.vxMetersPerSecond,
        currentSpeeds.vyMetersPerSecond
      ) < hypot(tolerance.x, tolerance.y)
//      && currentSpeeds.omegaRadiansPerSecond < tolera.`in`(RadiansPerSecond)
  }
}
