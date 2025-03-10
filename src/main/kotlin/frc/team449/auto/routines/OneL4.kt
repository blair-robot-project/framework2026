package frc.team449.auto.routines

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

class OneL4(
  robot: Robot,
  isRedAlliance: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      poseSubsystem = robot.poseSubsystem,
      parallelEventMap = hashMapOf(
        0 to robot.superstructureManager.requestGoal(SuperstructureGoal.STOW),
      ),
//      stopEventMap = hashMapOf(
//        1 to scoreL4(robot, if (isRedAlliance) FieldConstants.ReefSide.RIGHT else FieldConstants.ReefSide.LEFT)
//          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
//      ),
      debug = false,
      timeout = 0.0,
      stopDriveAfterTraj = false
    )

  private fun premoveL4(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
  }

  private fun scoreL4(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestL4()
      .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.5))
      .andThen(WaitCommand(1.75))
      .andThen(robot.intake.outtakeCoral())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.stop())
      .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE))
  }

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRedAlliance) {
      AutoUtil.transformForRedAlliance(
        ChoreoTrajectory.createTrajectory(arrayListOf("1"), "OneL4")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("1"), "OneL4")
    }
}
