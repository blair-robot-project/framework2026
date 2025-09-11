package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal

open class Routines(
  val robot: Robot
) {

  private val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    { sample: SwerveSample -> robot.drive.followTrajectory(robot, sample) },
    true,
    robot.drive
  )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  /** link to starting position on the field: https://docs.google.com/document/d/1SOzIJDgJ0GRSVnNTcBhaFfltvHw0IjJTEUsAZbI2hW4/edit?usp=sharing  **/
  /** left and right are from the driver's pov **/

  fun taxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("prev/taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }

  fun middleRoutine(): AutoRoutine {
    val middleRoutine = autoFactory.newRoutine("one l4 ")
    val forward = middleRoutine.trajectory("OneL4/1")
    val end = middleRoutine.trajectory("OneL4/2")

    middleRoutine.active().onTrue(
      Commands.sequence(
        forward.resetOdometry(),
        forward.cmd().alongWith(
          WaitCommand(0.5).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
          )
        )
      )
    )

    forward.done().onTrue(
      Commands.sequence(
        end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
        robot.drive.driveStop()
      )

    )
    return middleRoutine
  }

  /**Ground Intake Autos**/

  private fun getScoreCommand(reefLevel: Int): (FieldConstants.ReefSide) -> Command {
    return when (reefLevel) {
      2 -> { side: FieldConstants.ReefSide -> scoreL2PivotDirectional(side) }
      4 -> { side: FieldConstants.ReefSide -> scoreL4PivotSideDirectional(side) }
      else -> { side: FieldConstants.ReefSide -> scoreL4PivotSideDirectional(side) }
    }
  }

  private fun getPremoveCommand(reefLevel: Int, waitTime: Double = 0.0): Command {
    return when (reefLevel) {
      2 -> Commands.deadline(
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
      4 -> Commands.deadline(
        Commands.sequence(
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
            .withDeadline(WaitCommand(waitTime)),
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
        )
      )
      else -> Commands.parallel(
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW),
      )
    }
  }

//  // pass in "l" or "r" for direction
//  private fun ground3Point5(direction: String, reefLevel: IntArray): AutoRoutine {
//    val routine = autoFactory.newRoutine("3.5 Ground 3L4 ${if (direction == "r") "Right" else "Left"}")
//    val preloadScore = routine.trajectory("GroundThreeHalf/1$direction")
//    val firstPickup = routine.trajectory("GroundThreeHalf/2$direction")
//    val firstScore = routine.trajectory("GroundThreeHalf/3$direction")
//    val secondPickup = routine.trajectory("GroundThreeHalf/4$direction")
//    val secondScore = routine.trajectory("GroundThreeHalf/5$direction")
//    val thirdPickup = routine.trajectory("GroundThreeHalf/6$direction")
//    val thirdScore = routine.trajectory("GroundThreeHalf/7$direction") // give up on 4 piece
//
//    val firstPickupTime = if (direction == "l") 3.0 else 2.8
//    val secondPickupTime = 2.7 // same on both
//    val missNearPickupTime = 3.0 // same on both
//
//    val missNearPickup = routine.trajectory("GroundThreeHalf/failnear1$direction")
//    val missNearScore = routine.trajectory("GroundThreeHalf/failnear2$direction")
//    val missNearSecondPickup = routine.trajectory("GroundThreeHalf/failnear3$direction")
//    val missNearSecondScore = routine.trajectory("GroundThreeHalf/failnear4$direction") // not possible to 3 on miss
//    val missMidPickup = routine.trajectory("GroundThreeHalf/failmid1$direction")
//    val missMidScore = routine.trajectory("GroundThreeHalf/failmid2$direction") // not possible to 2 on double miss
//
//    routine.active().onTrue(
//      Commands.sequence(
//        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
//        preloadScore.cmd().alongWith(
//          robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[0]))
//            .withDeadline(WaitCommand(1.5))
//        )
//      )
//    )
//
//    preloadScore.done().onTrue(
//      Commands.sequence(
//        getScoreCommand(reefLevel[0]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
//        firstPickup.cmd().alongWith(intake()).withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),
//        ConditionalCommand(
//          Commands.sequence(
//            firstScore.cmd().alongWith(
//              WaitCommand(0.52).andThen(
//                robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[1]))
//              )
//            )
//          ),
//
//          // if we miss picking up the first coral, do a backup routine
//          Commands.sequence(
//            missNearPickup.cmd().alongWith(
//              robot.intake.outtakeL1().withTimeout(1.0)
//                .andThen(intake())
//            ).withTimeout(missNearPickupTime + AutoConstants.INTAKE_TIMEOUT),
//            ConditionalCommand(
//              missNearScore.cmd().alongWith(
//                WaitCommand(0.52).andThen(
//                  robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[1]))
//                )
//              ).andThen(robot.drive.driveStop()),
//              missMidPickup.cmd().alongWith(robot.intake.outtakeL1().withTimeout(1.0).andThen(intake())) // 1.5 on double miss
//            ) { robot.intake.coralDetected() || !RobotBase.isReal() }
//          )
//
//        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
//      )
//    )
//
//    // backup routines
//    missNearScore.done().onTrue(
//      Commands.sequence(
//        getScoreCommand(reefLevel[1]).invoke(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
//        missNearSecondPickup.cmd().alongWith(intake()),
//      )
//    )
//
//    firstScore.done().onTrue(
//      Commands.sequence(
//        getScoreCommand(reefLevel[1]).invoke(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
//        secondPickup.cmd().alongWith(intake()).withTimeout(secondPickupTime + AutoConstants.INTAKE_TIMEOUT),
//        robot.drive.driveStop(),
//        ConditionalCommand(
//          secondScore.cmd().alongWith(
//            WaitCommand(0.52).andThen(
//              robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[2]))
//            )
//          ),
//          missMidPickup.cmd().alongWith(robot.intake.outtakeL1().withTimeout(1.0).andThen(intake()))
//        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
//      )
//    )
//
//    secondScore.done().onTrue(
//      Commands.sequence(
//        getScoreCommand(reefLevel[2]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
//        intake().alongWith(thirdPickup.cmd()),
//        robot.drive.driveStop(),
//      )
//    )
//
//    return routine
//  }
//
//  // three l4 starting from a side then the back two reefs then half
//  fun rightGround3L4Half(): AutoRoutine {
//    return ground3Point5("r", intArrayOf(4, 4, 4))
//  }
//
//  fun leftGround3L4Half(): AutoRoutine {
//    return ground3Point5("l", intArrayOf(4, 4, 4))
//  }

  // back l4 and then sides 2 l4
  private fun threeL4(direction: String): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4 ${if (direction == "r") "Right" else "Left"}")
    val preloadScore = middlesides.trajectory("middleSides/1$direction")
    val firstPickup = middlesides.trajectory("middleSides/2$direction")
    val firstPresagedScore = middlesides.trajectory("middleSides/3$direction")
    val secondPickup = middlesides.trajectory("middleSides/4$direction")
    val secondPresagedScore = middlesides.trajectory("middleSides/5$direction")
    val end = middlesides.trajectory("middleSides/end$direction")

    middlesides.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry(),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(intake()),
        robot.drive.driveStop(),
        firstPresagedScore.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
          )
        )
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(intake()),
        robot.drive.driveStop(),
        secondPresagedScore.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
          )
        )
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      )
    )

    return middlesides
  }

  fun left3L4(): AutoRoutine {
    return threeL4("l")
  }

  fun right3L4(): AutoRoutine {
    return threeL4("r")
  }

  private fun groundBack2L4L2(direction: String, reefLevels: IntArray): AutoRoutine {
    val routine = autoFactory.newRoutine("2 l4 and l2 ${if (direction == "r") "Right" else "Left"}")
    val scorePreloadB = routine.trajectory("TwoL4L2/1$direction")
    val pickupMiddle = routine.trajectory("TwoL4L2/2$direction")
    val scoreMiddleA = routine.trajectory("TwoL4L2/3$direction")
    val pickupLeft = routine.trajectory("TwoL4L2/4$direction")
    val scoreRightB = routine.trajectory("TwoL4L2/5$direction")
    val pickupRight = routine.trajectory("TwoL4L2/6$direction")
    val scoreLeftA = routine.trajectory("TwoL4L2/7$direction")

    val firstPickupTime = 3.1 // same on both
    val secondPickupTime = 2.8 // same on both
    val thirdPickupTime = 3.0

    val missMidPickup = routine.trajectory("TwoL4L2/failmid1$direction")
    val missMidScore = routine.trajectory("TwoL4L2/failmid2$direction")
    val missMidSecondPickup = routine.trajectory("TwoL4L2/failmid3$direction")
    val missMidSecondScore = routine.trajectory("TwoL4L2/failmid4$direction") // second score impossible on miss

    val missFarPickup = routine.trajectory("TwoL4L2/failfar1$direction")
    val missFarScore = routine.trajectory("TwoL4L2/failfar2$direction")

    routine.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry(),
        scorePreloadB.cmd().alongWith(getPremoveCommand(reefLevels[0], 1.65))
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scorePiece(),
        pickupMiddle.cmd()
          .alongWith(intake())
          .andThen(robot.drive.driveStop())
          .withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),
        ConditionalCommand(
          scoreMiddleA.cmd()
            .alongWith(getPremoveCommand(reefLevels[1], 0.65)),
          Commands.sequence(
            missMidPickup.cmd()
              .alongWith(
                intake()
              )
              .andThen(robot.drive.driveStop())
              .withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),
            ConditionalCommand(
              missMidScore.cmd()
                .alongWith(getPremoveCommand(reefLevels[1], 0.85)),
              Commands.sequence(
                robot.drive.driveStop(),
                missFarPickup.cmd()
                  .alongWith(
                    intake()
                  )
                  .andThen(robot.drive.driveStop()),
                missFarScore.cmd().alongWith(
                  getPremoveCommand(reefLevels[1], 0.85)
                ),
                robot.drive.driveStop(),
                scorePiece(),
                robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
              )
            ) { !RobotBase.isReal() }

          )
        ) { !RobotBase.isReal() }
      )
    )

    // first miss backup
    missMidScore.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scorePiece(),
        missMidSecondPickup.cmd()
          .alongWith(intake())
          .andThen(robot.drive.driveStop()),
        missMidSecondScore.cmd()
          .alongWith(getPremoveCommand(reefLevels[2])),
        robot.drive.driveStop(),
        scorePiece(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scorePiece(),
        pickupLeft.cmd()
          .alongWith(intake())
          .andThen(robot.drive.driveStop())
          .withTimeout(secondPickupTime + AutoConstants.INTAKE_TIMEOUT),
        ConditionalCommand(
          scoreRightB.cmd()
            .alongWith(getPremoveCommand(reefLevels[2])),
          Commands.sequence(
            missFarPickup.cmd()
              .alongWith(
                intake()
              )
              .andThen(robot.drive.driveStop()),
            missFarScore.cmd().alongWith(
              getPremoveCommand(reefLevels[2], 0.85)
            ),
            robot.drive.driveStop(),
            scorePiece()
          )
        ) { !RobotBase.isReal() }

      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          robot.drive.driveStop(),
          scorePiece(),
          pickupRight.cmd()
            .alongWith(intake())
            .andThen(robot.drive.driveStop())
            .withTimeout(thirdPickupTime + AutoConstants.INTAKE_TIMEOUT),
          scoreLeftA.cmd()
            .alongWith(getPremoveCommand(reefLevels[3]))
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          robot.drive.driveStop(),
          scorePiece(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
        )
      )

    return routine
  }

  fun rightGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("r", intArrayOf(4, 4, 2, 2))
  }

  fun leftGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("l", intArrayOf(4, 4, 2, 2))
  }

  // Elevator is cooked!
  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
//    autoChooser.addRoutine("Right 3.5 L4 Back & Sides", this::rightGround3L4Half)
//    autoChooser.addRoutine("Left 3.5 L4 Back & Sides", this::leftGround3L4Half)

    autoChooser.addRoutine("2 l4 and l2 Right", this::rightGroundBack2L4L2)
    autoChooser.addRoutine("2 l4 and l2 Left", this::leftGroundBack2L4L2)

    autoChooser.addRoutine("Left 3 L4 Middle & Sides", this::left3L4)
    autoChooser.addRoutine("Right 3 L4 Middle & Sides", this::right3L4)

    autoChooser.addRoutine("Taxi", this::taxi)

    autoChooser.addRoutine("Center 1 L4", this::middleRoutine)
  }

  private fun scoreL4PivotSideDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      .alongWith(
        PrintCommand("Actually reached auto tolerance!")
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(
        WaitUntilCommand { !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
  }

  private fun scorePiece(): Command {
    return WaitUntilCommand { robot.superstructureManager.isAtPos() }
      .andThen(WaitCommand(0.15))
      .andThen(PrintCommand("Outook Piece!"))
      .andThen(PrintCommand("Piece left the robot!"))
      .andThen(WaitCommand(0.050))
  }

  private fun scoreL2PivotDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      .alongWith(
        PrintCommand("Actually reached auto tolerance!")
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(
        WaitUntilCommand { !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
  }

  private fun intake(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      .andThen(
        WaitUntilCommand { !RobotBase.isReal() }
      )
  }
}
