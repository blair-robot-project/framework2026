package frc.team449.hardwaremanagers.superstructure

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import frc.team449.config.RobotConstants
import frc.team449.input.HolonomicOI

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
    val maxSpeed: LinearVelocity,
    val maxAccel: LinearAcceleration,
    val maxRotSpeed: AngularVelocity
  )

  fun applyDriveDynamics(oi: HolonomicOI, dynamics: DriveDynamics) {
    oi.maxLinearSpeed = dynamics.maxSpeed
    oi.maxAccel = dynamics.maxAccel
    oi.maxRotationalSpeed = dynamics.maxRotSpeed
  }
}
