package frc.team449.sim

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import org.dyn4j.geometry.Ellipse
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
import org.ironmaple.simulation.gamepieces.GamePieceProjectile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveSim
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.utils.FieldMirroringUtils

/**
 * Creates a flying Lunite Football
 * @param robotPosition Position of the robot, relative to the field
 * @param relativeShooterPosition Position of the shooter, relative to the robot
 * @param
 */
class Lunite (
  robotPosition: Translation2d ,
  shooterPositionOnRobot: Translation2d ,
  chassisSpeedsFieldRelative: ChassisSpeeds,
  shooterFacing: Rotation2d,
  initialHeight: Distance,
  launchingSpeed: LinearVelocity,
  shooterAngle: Angle
) : GamePieceProjectile(LUNITE_INFO, robotPosition, shooterPositionOnRobot, chassisSpeedsFieldRelative, shooterFacing, initialHeight, launchingSpeed, shooterAngle) {

  // Lunite Constants
  companion object {
    val LUNITE_INFO: GamePieceOnFieldSimulation.GamePieceInfo = GamePieceOnFieldSimulation.GamePieceInfo(
      "Lunite",
      Ellipse(Inches.of(7.0).`in`(Meters), Inches.of(4.0).`in`(Meters)),
      Inches.of(4.0),
      Pounds.of(3.62),
      0.05,
      0.5,
      0.65
    )

    val SOLAR_CORE: Translation3d = Translation3d()
    val SOLAR_CORE_TOLERANCE: Translation3d = Translation3d()
    val SHEILD_GENERATOR: Translation3d = Translation3d()
    val SHEILD_GENERATOR_TOLERANCE: Translation3d = Translation3d()

    // Launch Lunite
    fun launchLunite(robot: Robot, shooter: Translation3d, angle: Angle, velocity: LinearVelocity) {
      if (isReal())
        return
      val driveSim = robot.drive as SwerveSim
      SimulatedArena.getInstance().addGamePieceProjectile(Lunite(
          driveSim.odometryPose.translation,
        Translation2d(shooter.x,shooter.y),
          driveSim.currentSpeeds,
          driveSim.currHeading,
          Meters.of(shooter.z),
          velocity,
          angle
        )
      )
    }
  }

  // Lunite Config
  init {
    super.withTouchGroundHeight(Inches.of(2.0).`in`(Meters))
    super.enableBecomesGamePieceOnFieldAfterTouchGround()
    // Score
    super.withTargetPosition { FieldMirroringUtils.toCurrentAllianceTranslation(SOLAR_CORE) }
      .withTargetTolerance(SOLAR_CORE_TOLERANCE)
      .withHitTargetCallBack {
        Scoreboard.score(
          if (DriverStation.isAutonomous()) 7 else 5,
          "Solar Core"
        )
        this.cleanUp()
      }
    super.withTargetPosition { FieldMirroringUtils.toCurrentAllianceTranslation(SHEILD_GENERATOR) }
      .withTargetTolerance(SHEILD_GENERATOR_TOLERANCE)
      .withHitTargetCallBack {
        Scoreboard.score(
          if (DriverStation.isAutonomous()) 4 else 2,
          "Sheild Generator"
        )
        this.cleanUp()
      }
  }

}