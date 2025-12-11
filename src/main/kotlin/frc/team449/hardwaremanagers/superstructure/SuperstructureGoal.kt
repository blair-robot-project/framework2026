package frc.team449.hardwaremanagers.superstructure

import frc.team449.config.RobotConstants
import frc.team449.hardwaremanagers.drive.DriveDynamics

object SuperstructureGoal {

  val STOW = SuperstructureState(
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Stow"
  )

  data class SuperstructureState(
    val driveDynamics: DriveDynamics,
    val name: String
  )
}
