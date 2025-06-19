package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
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

  // request algae ground intake
  // pivot has to get in set point before moving wrist for ground intake
// wait for pivot and elevator to get at setpoint and move wrist

  fun requestGroundIntake(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ lastGoal = goal }))
      .andThen(
        ConditionalCommand(
          // if ground algae
          Commands.sequence(
            Commands.parallel(
              elevator.setPosition(goal.elevator.`in`(Meters)),
              pivot.setPosition(goal.pivot.`in`(Radians))
            ),
            WaitUntilCommand { wrist.elevatorReady() && wrist.pivotReady() }, // TODO: check wrist constants for real values

            wrist.setPosition(goal.wrist.`in`(Radians)),
            WaitUntilCommand { pivot.atSetpoint() },

            pivot.hold().onlyIf { pivot.atSetpoint() },
            elevator.hold().onlyIf { elevator.atSetpoint() },
            wrist.hold().onlyIf { wrist.atSetpoint() },
            WaitUntilCommand { elevator.atSetpoint() && pivot.atSetpoint() },
            pivot.hold(),
            elevator.hold(),
            WaitUntilCommand { wrist.atSetpoint() },
            Commands.parallel(
              pivot.hold(),
              wrist.hold(),
              elevator.hold()
            )
          ),

          // if ground coral
          Commands.sequence(
            InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
            Commands.parallel(
              elevator.setPosition(goal.elevator.`in`(Meters)),
              pivot.setPosition(goal.pivot.`in`(Radians))
            ),
            WaitUntilCommand { wrist.elevatorReady() && wrist.pivotReady() }, // TODO: check wrist constants for real values and maybe

            wrist.setPosition(goal.wrist.`in`(Radians)),
            WaitUntilCommand { pivot.atSetpoint() },

            pivot.hold().onlyIf { pivot.atSetpoint() },
            elevator.hold().onlyIf { elevator.atSetpoint() },
            wrist.hold().onlyIf { wrist.atSetpoint() },
            WaitUntilCommand { elevator.atSetpoint() && pivot.atSetpoint() },
            pivot.hold(),
            elevator.hold(),
            WaitUntilCommand { wrist.atSetpoint() },
            Commands.parallel(
              pivot.hold(),
              wrist.hold(),
              elevator.hold()
            )
          )
        ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
      )
      .andThen(InstantCommand({ prevGoal = goal }))
      .andThen(InstantCommand({ ready = true }))
  }

  private fun retractFromHigh(goal: SuperstructureGoal.SuperstructureState, wristPremoveTime: Double): Command {
    return Commands.sequence(
      wrist.setPosition(Units.degreesToRadians(90.0))
        .onlyIf { prevGoal == SuperstructureGoal.L4 || prevGoal == SuperstructureGoal.NET },
      wrist.setPosition(Units.degreesToRadians(-70.0))
        .onlyIf { prevGoal == SuperstructureGoal.L4_PIVOT || prevGoal == SuperstructureGoal.NET_PIVOT },
      WaitUntilCommand { wrist.positionSupplier.get() > Units.degreesToRadians(20.0) }
        .onlyIf { prevGoal == SuperstructureGoal.L4 || prevGoal == SuperstructureGoal.NET },
      WaitUntilCommand { wrist.positionSupplier.get() < Units.degreesToRadians(10.0) }
        .onlyIf { prevGoal == SuperstructureGoal.L4_PIVOT || prevGoal == SuperstructureGoal.NET_PIVOT },
      elevator.setPosition(goal.elevator.`in`(Meters)).alongWith(
        WaitCommand(wristPremoveTime).andThen(wrist.setPosition(goal.wrist.`in`(Radians)))),
      WaitUntilCommand { elevator.pivotReady() },
      pivot.setPosition(goal.pivot.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
      InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
      holdAll()
    )
  }

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState, wristPremoveTime: Double = 0.25): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ lastGoal = goal }))
      .andThen(
        ConditionalCommand(
          // if extending
          Commands.sequence(
            InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
            ConditionalCommand(
              // to algae ground
              // move pivot before moving wrist
              Commands.sequence(
                pivot.setPosition(goal.pivot.`in`(Radians)),
                WaitUntilCommand { pivot.elevatorReady() },
                elevator.setPosition(goal.elevator.`in`(Meters)),
                // wait until pivot is almost at setpoint before moving wrist
                WaitUntilCommand { pivot.atSetpoint(Units.degreesToRadians(5.0)) },
                wrist.setPosition(goal.wrist.`in`(Radians)),
                WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
                pivot.hold().onlyIf { pivot.atSetpoint() },
                wrist.hold().onlyIf { wrist.atSetpoint() },
                WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
                pivot.hold(),
                wrist.hold(),
                WaitUntilCommand { elevator.atSetpoint() },
                holdAll()
              ),
              // regular extension
              Commands.sequence(
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
            ) { goal == SuperstructureGoal.ALGAE_GROUND  && prevGoal == SuperstructureGoal.GROUND_INTAKE_CORAL }
          ),

          // if retracting
          ConditionalCommand(
            // this function will check if we're retracting from l4/net and make necessary adjustments
            retractFromHigh(goal, wristPremoveTime),
            Commands.sequence(
              ConditionalCommand(
                // going to ground coral
                Commands.sequence(
                  wrist.setPosition(goal.wrist.`in`(Radians)),
                  // wait until wrist is almost at setpoint before moving pivot
                  WaitUntilCommand { (wrist.atSetpoint(Units.degreesToRadians(6.0))) },
                  pivot.setPosition(goal.pivot.`in`(Radians)),
                  WaitUntilCommand { pivot.elevatorReady() },
                  elevator.setPosition(goal.elevator.`in`(Meters)),
                  WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
                  pivot.hold().onlyIf { pivot.atSetpoint() },
                  wrist.hold().onlyIf { wrist.atSetpoint() },
                  WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
                  pivot.hold(),
                  wrist.hold(),
                  WaitUntilCommand { elevator.atSetpoint() },
                  holdAll()
                ),

                Commands.sequence(
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
              ) { goal == SuperstructureGoal.GROUND_INTAKE_CORAL && prevGoal == SuperstructureGoal.ALGAE_GROUND }
            )
          ) { prevGoal == SuperstructureGoal.L4 || prevGoal == SuperstructureGoal.L4_PIVOT }
        ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
      )
      .andThen(InstantCommand({ prevGoal = goal }))
      .andThen(InstantCommand({ ready = true }))
  }

  fun requestHigh(goal: SuperstructureGoal.SuperstructureState = SuperstructureGoal.L4): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ lastGoal = goal }))
      .andThen(
        ConditionalCommand(
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
      )
      .andThen(InstantCommand({ prevGoal = goal }))
      .andThen(InstantCommand({ ready = true }))
  }

  fun isAtPos(): Boolean {
    return ready
  }

  fun lastRequestedGoal(): SuperstructureGoal.SuperstructureState {
    return lastGoal
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
