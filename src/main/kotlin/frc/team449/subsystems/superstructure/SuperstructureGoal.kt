package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.wrist.Wrist
import frc.team449.subsystems.superstructure.elevator.Elevator

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
    Degrees.of(57.197861026),
    Inches.of(0.0),
    Degrees.of(-95.035993817),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3 = SuperstructureState(
    Degrees.of(64.5),
    Inches.of(15.75),
    Degrees.of(-104.5),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4 = SuperstructureState(
    Degrees.of(73.676012622),
    Inches.of(45.823700787),
    Degrees.of(-139.015031), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val SUBSTATION_INTAKE = SuperstructureState(
    Degrees.of(64.5),
    Meters.of(0.0),
    Degrees.of(77.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val STOW = SuperstructureState(
    Degrees.of(40.0),
    Meters.of(0.0),
    Degrees.of(90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val CLIMB = SuperstructureState(
    Degrees.of(60.0),
    Meters.of(0.0),
    Degrees.of(-90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  /** Actually find these positions*/
  val L2_ALGAE_DESCORE = SuperstructureState(
    Degrees.of(60.0),
    Meters.of(0.0),
    Degrees.of(-90.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_ALGAE_DESCORE = SuperstructureState(
    Degrees.of(60.0),
    Meters.of(0.0),
    Degrees.of(-90.0),
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

  fun getPivotForward(pivotAngle : Double, elevatorDistance: Double, wristAngle : Double, changeInDegrees: Double) : SuperstructureState {
    return SuperstructureState(
      Degrees.of(Units.radiansToDegrees(pivotAngle) + changeInDegrees),
      Meters.of(elevatorDistance),
      Degrees.of(Units.radiansToDegrees(wristAngle)),
      DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
    )
  }

  fun getPivotBackwards(pivotAngle : Double, elevatorDistance: Double, wristAngle : Double, changeInDegrees: Double) : SuperstructureState {
    return SuperstructureState(
      Degrees.of(Units.radiansToDegrees(pivotAngle) - changeInDegrees),
      Meters.of(elevatorDistance),
      Degrees.of(Units.radiansToDegrees(wristAngle)),
      DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
    )
  }

  fun getWristForward(pivotAngle : Double, elevatorDistance: Double, wristAngle : Double, changeInDegrees: Double) : SuperstructureState {
    return SuperstructureState(
      Degrees.of(Units.radiansToDegrees(pivotAngle)),
      Meters.of(elevatorDistance),
      Degrees.of(Units.radiansToDegrees(wristAngle) + changeInDegrees),
      DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
    )
  }

  fun getElevatorUp(pivotAngle : Double, elevatorDistance: Double, wristAngle : Double, changeInCentimeters: Double) : SuperstructureState {
    return SuperstructureState(
      Degrees.of(Units.radiansToDegrees(pivotAngle)),
      Meters.of(elevatorDistance + changeInCentimeters),
      Degrees.of(Units.radiansToDegrees(wristAngle)),
      DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
    )
  }

  fun getElevatorDown(pivotAngle : Double, elevatorDistance: Double, wristAngle : Double, changeInCentimeters: Double) : SuperstructureState {
    return SuperstructureState(
      Degrees.of(Units.radiansToDegrees(pivotAngle)),
      Meters.of(elevatorDistance + changeInCentimeters),
      Degrees.of(Units.radiansToDegrees(wristAngle)),
      DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
    )
  }

  fun getWristBackwards(pivotAngle : Double, elevatorDistance: Double, wristAngle : Double, changeInDegrees: Double) : SuperstructureState {
    return SuperstructureState(
      Degrees.of(Units.radiansToDegrees(pivotAngle)),
      Meters.of(elevatorDistance),
      Degrees.of(Units.radiansToDegrees(wristAngle) - changeInDegrees),
      DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
    )
  }
}
