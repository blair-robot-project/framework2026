package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.wrist.Wrist
import frc.team449.subsystems.superstructure.wrist.WristConstants

class SuperstructureManager(
  private val elevator: Elevator,
  private val pivot: Pivot,
  private val wrist: Wrist,
  private val drive: SwerveDrive
) {

  private var lastGoal = SuperstructureGoal.STOW
  private var prevGoal = SuperstructureGoal.STOW
  private var ready = false

  private fun handleCoralToAlgaeGround(): Command {
    val goal = SuperstructureGoal.ALGAE_GROUND
    // move wrist forward with pivot to setpoint and then move wrist to setpoint
    return Commands.sequence(
      pivot.setPosition(goal.pivot.`in`(Radians))
        // move wrist 30 degrees forward to prevent crashing
        .alongWith(wrist.setPosition(wrist.positionSupplier.get() + 0.5)),
      // wait until pivot is almost at setpoint before moving wrist
      WaitUntilCommand { pivot.atSetpoint(Units.degreesToRadians(5.0)) },
      wrist.setPosition(goal.wrist.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      holdAll()
    )
  }

  private fun handleAlgaeToCoralGround(): Command {
    val goal = SuperstructureGoal.GROUND_INTAKE_CORAL
    // move wrist before moving pivot
    return Commands.sequence(
      wrist.setPosition(goal.wrist.`in`(Radians)),
      // wait until pivot is almost at setpoint before moving wrist
      WaitUntilCommand { wrist.atSetpoint(Units.degreesToRadians(5.0)) },
      pivot.setPosition(goal.pivot.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      holdAll()
    )
  }

  private fun handleExtension(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(
      Commands.parallel(
        wrist.setPosition(goal.wrist.`in`(Radians)),
        pivot.setPosition(goal.pivot.`in`(Radians))
      ),
      WaitUntilCommand { wrist.elevatorReady() && pivot.elevatorReady() },
      elevator.setPosition(goal.elevator.`in`(Meters)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      pivot.hold(),
      wrist.hold(),
      WaitUntilCommand { elevator.atSetpoint() },
      holdAll()
    )
  }

  private fun requestExtending(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(
      InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
      ConditionalCommand( // going from coral to algae ground comparison

        // if we're going from CORAL GROUND to ALGAE GROUND (elevator the same)
        handleCoralToAlgaeGround(),

        // we're not going to coral ground from algae ground
        ConditionalCommand( // going from algae ground to coral ground comparison

          // if we're going from ALGAE GROUND to CORAL GROUND (elevator the same)
          handleAlgaeToCoralGround(),

          // regular extension
          handleExtension(goal)

        ) { goal == SuperstructureGoal.GROUND_INTAKE_CORAL && prevGoal == SuperstructureGoal.ALGAE_GROUND }

      ) { goal == SuperstructureGoal.ALGAE_GROUND && prevGoal == SuperstructureGoal.GROUND_INTAKE_CORAL }
    )
  }

  private fun retractFromHigh(goal: SuperstructureGoal.SuperstructureState, wristPremoveTime: Double): Command {
    return Commands.sequence(
      wrist.setPosition(Units.degreesToRadians(60.0)) // TODO: check positions
        .onlyIf { prevGoal == SuperstructureGoal.L4 || prevGoal == SuperstructureGoal.NET },
      wrist.setPosition(Units.degreesToRadians(50.0)) // TODO: check positions
        .onlyIf { prevGoal == SuperstructureGoal.L4_PIVOT || prevGoal == SuperstructureGoal.NET_PIVOT },
      WaitUntilCommand { wrist.positionSupplier.get() > Units.degreesToRadians(20.0) }
        .onlyIf { prevGoal == SuperstructureGoal.L4 || prevGoal == SuperstructureGoal.NET },
      WaitUntilCommand { wrist.positionSupplier.get() < Units.degreesToRadians(10.0) }
        .onlyIf { prevGoal == SuperstructureGoal.L4_PIVOT || prevGoal == SuperstructureGoal.NET_PIVOT },
      elevator.setPosition(goal.elevator.`in`(Meters)).alongWith(
        WaitCommand(wristPremoveTime).andThen(wrist.setPosition(goal.wrist.`in`(Radians)))
      ),
      WaitUntilCommand { elevator.pivotReady() },
      pivot.setPosition(goal.pivot.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
      InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
      holdAll()
    )
  }

  private fun handleRetraction(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(
      elevator.setPosition(goal.elevator.`in`(Meters)),
      wrist.hold(),
      wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
        .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },
      WaitUntilCommand { elevator.pivotReady() },
      Commands.parallel(
        pivot.setPosition(goal.pivot.`in`(Radians)),
        wrist.setPosition(goal.wrist.`in`(Radians))
      ),
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
      InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
      holdAll()
    )
  }

  private fun handleL4PivotRetraction(): Command {
    val goal = SuperstructureGoal.STOW
    return Commands.sequence(
      wrist.setPosition(Units.degreesToRadians(50.0))
        // move pivot 10 degrees forward to ensure no collision with l4
        .alongWith(pivot.setPosition(pivot.positionSupplier.get() - 0.17)),
      WaitUntilCommand { wrist.positionSupplier.get() < Units.degreesToRadians(10.0) },
      elevator.setPosition(SuperstructureGoal.L3_PIVOT.elevator.`in`(Meters)),
      WaitUntilCommand { elevator.atSetpoint() },
      pivot.setPosition(goal.pivot.`in`(Radians)),
      wrist.setPosition(goal.wrist.`in`(Radians)),
      elevator.setPosition(goal.elevator.`in`(Meters)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      pivot.hold(),
      wrist.hold(),
      WaitUntilCommand { elevator.atSetpoint() },
      holdAll()
    )
  }

  private fun handleL3PivotRetraction(): Command {
    val goal = SuperstructureGoal.STOW
    return Commands.sequence(
      pivot.setPosition(goal.pivot.`in`(Radians)),
      // wait until pivot is almost at setpoint before moving wrist and elevator
      WaitUntilCommand { pivot.atSetpoint(Units.degreesToRadians(5.0)) },
      wrist.setPosition(goal.wrist.`in`(Radians)),
      WaitUntilCommand { wrist.elevatorReady() && pivot.elevatorReady() },
      elevator.setPosition(goal.elevator.`in`(Meters)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      pivot.hold(),
      wrist.hold(),
      WaitUntilCommand { elevator.atSetpoint() },
      holdAll()
    )
  }

  private fun requestRetracting(goal: SuperstructureGoal.SuperstructureState, wristPremoveTime: Double): Command {
    return ConditionalCommand(

      ConditionalCommand( // going to high goal comparison

        // if we scored l4 or l3 pivot side we need to watch out for climb
        ConditionalCommand(
          handleL3PivotRetraction(),
          handleL4PivotRetraction()
        ) { prevGoal == SuperstructureGoal.L3_PIVOT },

        // normal high retraction
        retractFromHigh(goal, wristPremoveTime)

      ) {
        prevGoal == SuperstructureGoal.L4_PIVOT || prevGoal == SuperstructureGoal.L3_PIVOT ||
          goal == SuperstructureGoal.STOW
      },

      // not coming from high, normal retraction
      handleRetraction(goal)

    ) {
      prevGoal == SuperstructureGoal.L4 || prevGoal == SuperstructureGoal.L4_PIVOT ||
        prevGoal == SuperstructureGoal.NET || prevGoal == SuperstructureGoal.NET_PIVOT
    }
  }

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState, wristPremoveTime: Double = 0.4): Command {
    // don't crash into reef with ground intake
    if ( goal == SuperstructureGoal.ALGAE_GROUND
      || goal == SuperstructureGoal.GROUND_INTAKE_CORAL
      && requestedOppSide() ) {
      println("\n\nHELOOOO\n\n")
      return requestGoal(SuperstructureGoal.STOW, wristPremoveTime)
    }

    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ lastGoal = goal }))
      .andThen(

        ConditionalCommand( // goal is high comparison
          // use special function for high goals
          requestHigh(goal),

          // not a high goal
          ConditionalCommand( // elevator height comparison

            // if extending or elevator the same
            requestExtending(goal),

            // if retracting
            requestRetracting(goal, wristPremoveTime)

          ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }

        ) {
          goal == SuperstructureGoal.L4 || goal == SuperstructureGoal.L4_PIVOT ||
            goal == SuperstructureGoal.NET || goal == SuperstructureGoal.NET_PIVOT
        }

      )
      .andThen(InstantCommand({ prevGoal = goal }))
      .andThen(InstantCommand({ ready = true }))
  }

  private fun requestHigh(goal: SuperstructureGoal.SuperstructureState = SuperstructureGoal.L4): Command {
    return ConditionalCommand(
      // if extending
      Commands.sequence(
        Commands.parallel(
          wrist.setPosition(goal.wrist.`in`(Radians)),
          pivot.setPosition(goal.pivot.`in`(Radians))
        ),
        WaitUntilCommand { wrist.elevatorReady() && pivot.elevatorReady() },
        elevator.setPositionCarriage(goal.elevator.`in`(Meters)),
        WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
        pivot.hold().onlyIf { pivot.atSetpoint() },
        wrist.hold().onlyIf { wrist.atSetpoint() },
        WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
        pivot.hold(),
        wrist.hold(),
        WaitUntilCommand { elevator.atSetpoint() },
        Commands.parallel(
          pivot.hold(),
          wrist.hold(),
          elevator.holdCarriage()
        )
      ),

      // if retracting
      Commands.sequence(
        elevator.setPositionCarriage(goal.elevator.`in`(Meters)),
        wrist.hold(),
        wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
          .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },
        WaitUntilCommand { elevator.pivotReady() },
        Commands.parallel(
          pivot.setPosition(goal.pivot.`in`(Radians)),
          wrist.setPosition(goal.wrist.`in`(Radians))
        ),
        WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
        Commands.parallel(
          pivot.hold(),
          wrist.hold(),
          elevator.holdCarriage()
        )
      )
    ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
  }

  fun isAtPos(): Boolean {
    return ready
  }

  fun lastRequestedGoal(): SuperstructureGoal.SuperstructureState {
    return lastGoal
  }

  private fun requestedOppSide(): Boolean {
    return (
      lastGoal == SuperstructureGoal.L1 ||
        lastGoal == SuperstructureGoal.L2 ||
        lastGoal == SuperstructureGoal.L3 ||
        lastGoal == SuperstructureGoal.L4 ||
        lastGoal == SuperstructureGoal.NET
      )
  }

  fun requestedPivotSide(): Boolean {
    return lastGoal == SuperstructureGoal.L2_PIVOT ||
      lastGoal == SuperstructureGoal.L3_PIVOT ||
      lastGoal == SuperstructureGoal.L4_PIVOT
  }

  fun intookAlgae(): Boolean {
    return lastGoal == SuperstructureGoal.ALGAE_GROUND ||
      lastGoal == SuperstructureGoal.L2_ALGAE_INTAKE ||
      lastGoal == SuperstructureGoal.L3_ALGAE_INTAKE
  }

  private fun holdAll(): Command {
    return Commands.parallel(
      pivot.hold(),
      wrist.hold(),
      elevator.hold()
    )
  }

  companion object {
    fun createSuperstructureManager(robot: Robot): SuperstructureManager {
      return SuperstructureManager(
        robot.elevator,
        robot.pivot,
        robot.wrist,
        robot.drive
      )
    }
  }
}
