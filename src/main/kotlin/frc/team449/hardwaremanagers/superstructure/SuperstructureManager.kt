package frc.team449.hardwaremanagers.superstructure

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot

@Logged
object SuperstructureManager {

  private var requestedGoal = SuperstructureGoal.STOW
  private var lastCompletedGoal = SuperstructureGoal.STOW
  private var ready = false

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(Robot.holonomicOi, goal.driveDynamics) })
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
}
