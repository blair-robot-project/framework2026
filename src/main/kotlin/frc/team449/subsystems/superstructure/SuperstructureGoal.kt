package frc.team449.subsystems.superstructure

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive

object SuperstructureGoal {
  /** TODO: All placeholder guesses, need actual values */
  private const val SCORING_SPEED = 2.6329
  private const val SCORING_ACCEL = 12.5

  private const val GROUND_INTAKE_SPEED = 3.5804

  private val MIN_ELEVATOR_HEIGHT = Inches.of(-0.35)

  val STOW = SuperstructureState(
    Degrees.of(40.0),
    MIN_ELEVATOR_HEIGHT,
    Degrees.of(90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L1 = SuperstructureState(
    Degrees.of(31.22),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-1.347656),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2 = SuperstructureState(
    Radians.of(-0.0168),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(2.1213),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3 = SuperstructureState(
    Radians.of(0.892578),
    Meters.of(0.21777),
    Radians.of(1.0114746),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4 = SuperstructureState(
    Radians.of(1.208008),
    Meters.of(0.8537597),
    Radians.of(0.256591), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PREMOVE = SuperstructureState(
    L4.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2_PIVOT = SuperstructureState(
    Radians.of(1.211914),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(1.503662),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_PIVOT = SuperstructureState(
    Radians.of(1.345947),
    Meters.of(0.184082),
    Radians.of(1.9582519),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PIVOT = SuperstructureState(
    Radians.of(1.45019), // 83 deg
    Meters.of(0.885253),
    Radians.of(2.33081), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val GROUND_INTAKE_CORAL = SuperstructureState(
    Radian.of(-0.02239 - 0.01),
    Inches.of(0.0),
    Radians.of(-0.335 - 0.05 + 0.005 + 0.005),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val CLIMB_BEFORE = SuperstructureState(
    Degrees.of(90.0),
    Meters.of(0.178891),
    Degrees.of(-70.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

//  // OLD L2 ALGAE DESCORE
//  val L2_ALGAE_DESCORE = SuperstructureState(
//    Degrees.of(53.449538),
//    Inches.of(7.694820-2)- MIN_ELEVATOR_HEIGHT,
//    Degrees.of(-90.0), // true angle is -84.87
//    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
//  )
//
//  // OLD L3 ALGAE DESCORE
//  val L3_ALGAE_DESCORE = SuperstructureState(
//    Radians.of(0.958984),
//    Meters.of(0.291016),
//    Radians.of(-1.064941),
//    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
//  )

  val L4_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    L4_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val NET = SuperstructureState(
    Degrees.of(80.297259),
    Inches.of(51.653671 - 2) - MIN_ELEVATOR_HEIGHT,
    Degrees.of(76.644829),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val NET_PREMOVE = SuperstructureState(
    NET.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND NET PIVOT POSE
  val NET_PIVOT = SuperstructureState(
    Radians.of(80.297259 * Math.PI / 180),
    Inches.of(49.121867 - 2) - MIN_ELEVATOR_HEIGHT,
    Radians.of((180 - 149.437262) * Math.PI / 180), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND PIVOT FOR PREMOVE STATE
  val NET_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    NET_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND ALGAE GROUND INTAKE POSE
  val ALGAE_GROUND = SuperstructureState(
    Radian.of(-0.333740),
    Inches.of(0.0),
    Radians.of(-1.477050 + 0.02),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND ALGAE L2 INTAKE POSE
  val L2_ALGAE_INTAKE = SuperstructureState(
    Degrees.of(53.449538),
    Inches.of(7.694820 - 2) - MIN_ELEVATOR_HEIGHT,
    Degrees.of(-90.0), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND ALGAE L3 INTAKE POSE
  val L3_ALGAE_INTAKE = SuperstructureState(
    Degrees.of(64.804651),
    Inches.of(20.926871 - 2) - MIN_ELEVATOR_HEIGHT,
    Degrees.of(-90.0), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val PROC = SuperstructureState(
    Radians.of(-0.00239),
    Inches.of(0.0),
    Degrees.of(-50.0),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  data class SuperstructureState(
    var pivot: Angle,
    var elevator: Distance,
    var wrist: Angle,
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
