package frc.team449.subsystems.superstructure

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive

object SuperstructureGoal {
  /** TODO: All placeholder guesses, need actual values */
  private const val SCORING_SPEED = 2.6672
  private const val SCORING_ACCEL = 12.5

  val L1 = SuperstructureState(
    Degrees.of(55.0),
    Meters.of(0.0),
    Degrees.of(-105.0),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2 = SuperstructureState(
    Degrees.of(58.38),
    Inches.of(0.3675),
    Degrees.of(-93.38),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3 = SuperstructureState(
    Degrees.of(69.5),
    Inches.of(15.75),
    Degrees.of(-104.5),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4 = SuperstructureState(
    Degrees.of(75.0),
    Inches.of(47.25),
    Degrees.of(-138.5),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val SUBSTATION_INTAKE = SuperstructureState(
    Degrees.of(64.5),
    Meters.of(0.0),
    Degrees.of(90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val STOW = SuperstructureState(
    Degrees.of(40.0),
    Meters.of(0.0),
    Degrees.of(90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L1_PREMOVE = SuperstructureState(
    L1.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2_PREMOVE = SuperstructureState(
    L2.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_PREMOVE = SuperstructureState(
    L3.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PREMOVE = SuperstructureState(
    L4.pivot,
    STOW.elevator,
    SUBSTATION_INTAKE.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  data class SuperstructureState(
    val pivot: Angle,
    val elevator: Distance,
    val wrist: Angle,
    val driveDynamics: DriveDynamics
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
