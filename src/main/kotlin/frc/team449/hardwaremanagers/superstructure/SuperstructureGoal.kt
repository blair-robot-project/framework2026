package frc.team449.hardwaremanagers.superstructure

import frc.team449.config.RobotConstants
import frc.team449.hardwaremanagers.drive.swerve.SwerveDrive

object SuperstructureGoal {

  val STOW = SuperstructureState(
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Stow"
  )

  data class SuperstructureState(
    val driveDynamics: DriveDynamics,
    val name: String
  )

  data class DriveDynamics(
    val maxSpeed: Double,
    val maxAccel: Double,
    val maxRotSpeed: Double
  )

  fun applyDriveDynamics(drive: SwerveDrive, dynamics: DriveDynamics) {
    drive.maxLinearSpeed = dynamics.maxSpeed
    drive.accel = dynamics.maxAccel
    drive.maxRotSpeed = dynamics.maxRotSpeed
  }
}
