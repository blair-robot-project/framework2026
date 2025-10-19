package frc.team449.subsystems.superstructure

import dev.doglog.DogLog
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem

@Logged
class SuperstructureManager(
  @NotLogged
  private val drive: SwerveDrive,
  @NotLogged
  private val poseSubsystem: PoseSubsystem
) {

  private var requestedGoal = SuperstructureGoal.STOW
  private var lastCompletedGoal = SuperstructureGoal.STOW
  private var ready = false

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ requestedGoal = goal }))
      .andThen(InstantCommand({ lastCompletedGoal = goal }))
      .andThen(InstantCommand({ ready = true }))
  }

  @Logged(name = "requested goal")
  fun getRequestedGoalForLog(): String {
    return requestedGoal.name
  }

  @Logged(name = "last completed goal")
  fun getLastCompletedGoalForLog(): String {
    return requestedGoal.name
  }

  fun isAtPos(): Boolean {
    return ready
  }

  fun lastCompletedGoal(): SuperstructureGoal.SuperstructureState {
    return lastCompletedGoal
  }

  fun logData() {
    DogLog.log("Superstructure/Current Requested Goal", requestedGoal.name)
    DogLog.log("Superstructure/Last Completed Goal", lastCompletedGoal.name)
  }

  companion object {
    fun createSuperstructureManager(robot: Robot): SuperstructureManager {
      return SuperstructureManager(
        robot.drive,
        robot.poseSubsystem
      )
    }
  }
}
