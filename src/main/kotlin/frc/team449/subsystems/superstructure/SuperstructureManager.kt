package frc.team449.subsystems.superstructure

import dev.doglog.DogLog
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.vision.PoseSubsystem

class SuperstructureManager(
  private val poseSubsystem: PoseSubsystem
) {

  private var requestedGoal = SuperstructureGoal.STOW
  private var lastCompletedGoal = SuperstructureGoal.STOW
  private var ready = false

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
