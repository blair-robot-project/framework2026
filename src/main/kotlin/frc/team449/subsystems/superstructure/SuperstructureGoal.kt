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

  val L1 = SuperstructureState(
    Radians.of(0.527538),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-1.347656),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2 = SuperstructureState(
    Radians.of(0.839355),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.193115),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3 = SuperstructureState(
    Radians.of(1.044059),
    Meters.of(0.249023),
    Radians.of(0.037354),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4 = SuperstructureState(
    Radians.of(1.249023),
    Meters.of(0.967773),
    Radians.of(-0.699707), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2_PIVOT = SuperstructureState(
    Radians.of(1.211914),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(1.503662),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_PIVOT = SuperstructureState(
    Radians.of(1.269531),
    Meters.of(0.148438),
    Radians.of(0.859375),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PIVOT = SuperstructureState(
    Radians.of(1.459688),
    Meters.of(1.030762),
    Radians.of(1.625244), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val SUBSTATION_INTAKE = SuperstructureState(
    Radians.of(1.035),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(1.419902),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val GROUND_INTAKE = SuperstructureState(
    Degrees.of(-5.15),
    Inches.of(-1.35),
    Degrees.of(-85.0), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val GROUND_INTAKE_HIGH = SuperstructureState(
    Degrees.of(-1.0),
    MIN_ELEVATOR_HEIGHT,
    Degrees.of(-85.5), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val SUBSTATION_INTAKE_CORAL_IN_FRONT = SuperstructureState(
    Degrees.of(63.329483462),
    MIN_ELEVATOR_HEIGHT,
    Degrees.of(79.88676053),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val STOW = SuperstructureState(
    Degrees.of(40.0),
    MIN_ELEVATOR_HEIGHT,
    Degrees.of(90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val CLIMB_BEFORE = SuperstructureState(
    Degrees.of(90.0),
    Meters.of(0.192891),
    Degrees.of(-70.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  //OLD L2 ALGAE DESCORE
  val L2_ALGAE_DESCORE = SuperstructureState(
    Degrees.of(42.188493899386),
    MIN_ELEVATOR_HEIGHT,
    Degrees.of(-57.88328506487372),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  //OLD L3 ALGAE DESCORE
  val L3_ALGAE_DESCORE = SuperstructureState(
    Radians.of(0.958984),
    Meters.of(0.291016),
    Radians.of(-1.064941),
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
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(65.0),
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_PREMOVE_PIVOT = SuperstructureState(
    L3_PIVOT.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    L4_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // only for after pivot side l4
  val PRE_GROUND = SuperstructureState(
    L4_PIVOT.pivot,
    GROUND_INTAKE.elevator,
    L4_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  //TODO: FIND NET POSE
  val NET = SuperstructureState(
    L4.pivot,
    L4.elevator,
    L4.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  //TODO: FIND ALGAE GROUND INTAKE POSE
  val ALGAE_GROUND = SuperstructureState(
    Degrees.of(-5.15),
    Inches.of(-1.35),
    Degrees.of(-85.0), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  //TODO: FIND ALGAE L2 INTAKE POSE
  val L2_ALGAE_INTAKE = SuperstructureState(
    Degrees.of(-5.15),
    Inches.of(-1.35),
    Degrees.of(-85.0), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  //TODO: FIND ALGAE L3 INTAKE POSE
  val L3_ALGAE_INTAKE = SuperstructureState(
    Degrees.of(-5.15),
    Inches.of(-1.35),
    Degrees.of(-85.0), // true angle is -84.87
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
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
