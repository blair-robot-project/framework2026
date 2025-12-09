package frc.team449.commands

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.team449.config.RobotConstants
import frc.team449.config.SwerveConstants
import frc.team449.hardwaremanagers.PoseSubsystem
import frc.team449.hardwaremanagers.drive.ChassisController
import frc.team449.hardwaremanagers.drive.SwerveChassis
import frc.team449.util.Clock
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

@Logged
class SwerveDriveCommand(
  private val drive: ChassisController,
  private val poseEstimator: PoseSubsystem,
  @NotLogged
  private val controller: XboxController,
  private val oi: ChassisController,
  private val fieldRelative: Boolean
) : Command() {
  // todo, refactor these to be updated on ds connect, so we don't have to poll it all the time
  private val allianceCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) PI else 0.0 }
  private val directionCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) -1.0 else 1.0 }

  var headingLock = false
  var pointLock = false
  var orbitPoint: Translation2d = Translation2d()
  var prevChassisSpeeds: ChassisSpeeds = ChassisSpeeds()

  private val timer = Timer()

  private val rotCtrl =
    PIDController(
      RobotConstants.SNAP_KP,
      RobotConstants.SNAP_KI,
      RobotConstants.SNAP_KD,
    )

  init {
    addRequirements(drive)
    rotCtrl.enableContinuousInput(-PI, PI)
    rotCtrl.setTolerance(RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD)
  }

  fun snapToAngle(angle: Rotation2d) {
    val desAngle = MathUtil.angleModulus(angle.radians + allianceCompensation.invoke())
    rotCtrl.setpoint = desAngle
    headingLock = true
    pointLock = false
    timer.reset()
  }

  fun snapToPoint(point: Translation2d) {
    orbitPoint = point
    headingLock = true
    pointLock = true
  }

  fun checkSnapToAngelTolerance(): Boolean {
    // if the setpoint is reached or the user is still demanding a significant rotation 0.5 seconds after starting the heading change
    return rotCtrl.atSetpoint() || (abs(controller.rightX) >= 0.3 && timer.hasElapsed(0.5))
  }

  fun exitSnapToAngle() {
    timer.stop()
    headingLock = false
    pointLock = false
  }

  /** Just a helper command factory to point at a given angle and stop the heading lock once you get into tolerance
   * If you want to customize when the heading lock is lifted, use the internal snapToAngle,
   *  checkSnapToAngleTolerance, and exitSnapToAngle functions */
  fun pointAtAngleCommand(angle: Rotation2d): Command =
    RunCommand({ snapToAngle(angle) })
      .until(rotCtrl::atSetpoint)
      .andThen(::exitSnapToAngle)

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
}
