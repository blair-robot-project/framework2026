package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

open class Routines(
  val robot: Robot
) {

  val autoFactory = AutoFactory(
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

  // right taxi
  fun rightTaxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("prev/taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }

  // left taxi
  fun leftTaxi(): AutoRoutine {
    val lTaxi: AutoRoutine = autoFactory.newRoutine("Left Taxi")
    val lTaxiTrajectory: AutoTrajectory = lTaxi.trajectory("prev/taxiLeft")
    lTaxi.active().onTrue(Commands.sequence(lTaxiTrajectory.resetOdometry(), lTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return lTaxi
  }

  // L1 coral at reef G
  fun l2reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L2 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("prev/middle test")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        Commands.parallel(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE),
          reefGTrajectory.cmd()
        )
      )

    )
    reefGTrajectory.done().onTrue(
      Commands.sequence(
        Commands.parallel(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2),
          SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))
        ),
        robot.drive.driveStop(),
        WaitCommand(0.15),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() }
      )
    )
    return g
  }

  // l4 at reef E reef D
  fun reefED(): AutoRoutine {
    val E_D: AutoRoutine = autoFactory.newRoutine("L4 reef E reef D")
    val reefETrajectory: AutoTrajectory = E_D.trajectory("prev/right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D.trajectory("prev/E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D.trajectory("prev/rightStation_D")
    E_D.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),

        // to reef
        reefETrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        reefEtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToDTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
      )
    )
    return E_D
  }
  fun reefEDhalf(): AutoRoutine {
    val E_DHalf: AutoRoutine = autoFactory.newRoutine("L4 reef E,D and half")
    val reefETrajectory: AutoTrajectory = E_DHalf.trajectory("prev/right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_DHalf.trajectory("prev/E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_DHalf.trajectory("prev/rightStation_D")
    val reefDToStationTrajectory: AutoTrajectory = E_DHalf.trajectory("prev/D_rightStation")
    E_DHalf.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),

        // to reef
        reefETrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        reefEtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToDTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        reefDToStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot)
      )
    )
    return E_DHalf
  }

  // l1 at reef J and then l4 at reef L
  fun reefJL(): AutoRoutine {
    val J_L: AutoRoutine = autoFactory.newRoutine("L1 reef J and L4 reef L")
    val reefJTrajectory: AutoTrajectory = J_L.trajectory("prev/left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_L.trajectory("prev/J_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_L.trajectory("prev/leftStation_L")
    J_L.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),

        // to reef
        reefJTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        reefJtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToLTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
      )
    )
    return J_L
  }

  // l1 at reef J and then l4 at reef L
  fun reefJLhalf(): AutoRoutine {
    val J_Lhalf: AutoRoutine = autoFactory.newRoutine("L1 reef J and L4 reef L")
    val reefJTrajectory: AutoTrajectory = J_Lhalf.trajectory("prev/left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_Lhalf.trajectory("prev/J_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_Lhalf.trajectory("prev/leftStation_L")
    val reefLtoStationTrajectory: AutoTrajectory = J_Lhalf.trajectory("prev/L_leftStation")

    J_Lhalf.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),

        // to reef
        reefJTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        reefJtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToLTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        reefLtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot)
      )
    )
    return J_Lhalf
  }

  // l4 at reef J, K, L
  fun reefJKL(): AutoRoutine {
    val J_K_L: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefJTrajectory: AutoTrajectory = J_K_L.trajectory("prev/left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_K_L.trajectory("prev/J_leftStation")
    val stationToKTrajectory: AutoTrajectory = J_K_L.trajectory("prev/leftStation_K")
    val reefKtoStationTrajectory: AutoTrajectory = J_K_L.trajectory("prev/K_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_K_L.trajectory("prev/leftStation_L")
    J_K_L.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),

        // to reef
        reefJTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        reefJtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToKTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        reefKtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToLTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT)

      )
    )

    return J_K_L
  }

  // l4 at reef J, K, L
  fun reefEDC(): AutoRoutine {
    val E_D_C: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefETrajectory: AutoTrajectory = E_D_C.trajectory("prev/right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D_C.trajectory("prev/E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D_C.trajectory("prev/rightStation_D")
    val reefDtoStationTrajectory: AutoTrajectory = E_D_C.trajectory("prev/D_rightStation")
    val stationToCTrajectory: AutoTrajectory = E_D_C.trajectory("prev/rightStation_C")
    E_D_C.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),

        // to reef
        reefETrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        reefEtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToDTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        reefDtoStationTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        ).andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        stationToCTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

      )
    )

    return E_D_C
  }
  fun l2Routine(): AutoRoutine {
    val l2autoRoutine = autoFactory.newRoutine("L4 Routine")

    val l2fTrajectory = l2autoRoutine.trajectory("prev/Go To L4F")
    val rightStationTrajectory = l2autoRoutine.trajectory("prev/Go To Right Station(2)")
    val l2ETrajectory = l2autoRoutine.trajectory("prev/Go to L4E")
    val rightStationTrajectory2 = l2autoRoutine.trajectory("prev/Go To Right Station(Again)")
    val l2DTrajectory = l2autoRoutine.trajectory("prev/Go To L4D")
    val rightStationTrajectory3 = l2autoRoutine.trajectory("prev/Go To Rightstation(3)")
    val l2CTrajectory = l2autoRoutine.trajectory("prev/Go To L4C")

    l2autoRoutine.active().onTrue(
      Commands.parallel(
        l2fTrajectory.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE)
      ).andThen(
        l2fTrajectory.cmd()
      )
    )

    l2fTrajectory.done().onTrue(
      Commands.sequence(
        SimpleReefAlign(
          robot.drive,
          robot.poseSubsystem,
          leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)
        )
          .alongWith(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
          ),
        robot.drive.driveStop().alongWith(
          robot.intake.outtakeCoral()
        ).andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        rightStationTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE))
      )
    )
    rightStationTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.intake.intakeCoral().andThen(
          WaitUntilCommand { robot.intake.coralDetected() },
          WaitCommand(0.15).onlyIf { RobotBase.isReal() },
          l2ETrajectory.cmd().alongWith(
            robot.superstructureManager.requestGoal(
              SuperstructureGoal.L2_PREMOVE
            )
          )
        )

      )

    )

    return l2autoRoutine
  }
  fun middleRoutine(): AutoRoutine {
    val middleRoutine = autoFactory.newRoutine("prev/middle Test")
    val test = middleRoutine.trajectory("prev/middle test")

    middleRoutine.active().onTrue(
      Commands.sequence(
        test.resetOdometry(),
        test.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
      )
    )

    test.done().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT), translationSpeedLim = 2.0, translationAccelLim = 1.4).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop(),
        WaitCommand(0.15),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return middleRoutine
  }

  fun americanRoutine(): AutoRoutine {
    val autoRoutine = autoFactory.newRoutine("L4 Routine")

    val l4ETrajectory = autoRoutine.trajectory("ThreeL4Right/1")
    val rightStationTrajectory = autoRoutine.trajectory("ThreeL4Right/2")
    val l4DTrajectory = autoRoutine.trajectory("ThreeL4Right/3")
    val rightStationTrajectory2 = autoRoutine.trajectory("ThreeL4Right/4")
    val l4CTrajectory = autoRoutine.trajectory("ThreeL4Right/5")

    autoRoutine.active().onTrue(
      Commands.sequence(
        l4ETrajectory.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
          .alongWith(
            robot.intake.holdCoral(),
            l4ETrajectory.cmd()
          )
      ),
    )

    l4ETrajectory.done().onTrue(
      ScoreL4(robot, FieldConstants.ReefSide.LEFT)
        .andThen(
          rightStationTrajectory.cmd()
            .alongWith(PremoveIntake(robot))
        )
    )

    rightStationTrajectory.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4DTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    l4DTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
        PremoveIntake(robot).alongWith(
          rightStationTrajectory2.cmd()
        )
      )
    )

    rightStationTrajectory2.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4CTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(
            WaitCommand(0.5)
          )
        )
      )
    )

    l4CTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return autoRoutine
  }
  fun LeftamericanRoutine(): AutoRoutine {
    val leftAutoRoutine = autoFactory.newRoutine("Left L4 Routine")

    val l4jTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/1")
    val lefStationTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/2")
    val l4kTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/3")
    val leftStationTraj2 = leftAutoRoutine.trajectory("ThreeL4Left/4")
    val l4LTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/5")

    leftAutoRoutine.active().onTrue(
      Commands.sequence(
        l4jTrajectory.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).alongWith(
          robot.intake.holdCoral(),
          l4jTrajectory.cmd()
        ),
        PrintCommand("Traveling to L4")
      )
    )

    l4jTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
        lefStationTrajectory.cmd().alongWith(PremoveIntake(robot))
      )
    )

    lefStationTrajectory.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4kTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    l4kTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        PremoveIntake(robot).alongWith(
          leftStationTraj2.cmd()
        )
      )
    )

    leftStationTraj2.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4LTrajectory.cmd()
          .alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    l4LTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )
    return leftAutoRoutine
  }

  /**Ground Intake Autos**/
  fun rightGround4L4(): AutoRoutine {
    val ground4piece = autoFactory.newRoutine("Left L4 Routine")
    val preloadScore = ground4piece.trajectory("Ground4L4right/1")
    val firstPickup = ground4piece.trajectory("Ground4L4right/2")
    val firstPresagedScore = ground4piece.trajectory("Ground4L4right/3")
    val secondPickup = ground4piece.trajectory("Ground4L4right/4")
    val secondPresagedScore = ground4piece.trajectory("Ground4L4right/5")
    val thirdPickup = ground4piece.trajectory("Ground4L4right/6")
    val thirdPresagedScore = ground4piece.trajectory("Ground4L4right/7")

    ground4piece.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).alongWith(
          robot.intake.holdCoral(),
          preloadScore.cmd()
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        firstPickup.cmd().alongWith(PremoveIntake(robot)) // replace with new ground intake command
      )
    )

    firstPickup.done().onTrue(
      Commands.sequence(
        Intake(robot),
        firstPresagedScore.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        PremoveIntake(robot).alongWith(
          secondPickup.cmd() // replace with new ground intake command
        )
      )
    )

    secondPickup.done().onTrue(
      Commands.sequence(
        Intake(robot),
        secondPresagedScore.cmd()
          .alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
        PremoveIntake(robot).alongWith(
          thirdPickup.cmd() // replace with new ground intake command
        )
      )
    )

    thirdPickup.done().onTrue(
      Commands.sequence(
        Intake(robot),
        thirdPresagedScore.cmd()
          .alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    thirdPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return ground4piece
  }

  fun rightCoralAlgae(): AutoRoutine {
    val twoL4removeAlgae = autoFactory.newRoutine("AB l4 + algae descore + A l3")
    val scorePreloadB = twoL4removeAlgae.trajectory("Left2L4Algae2L3/1")
    val pickupMiddle = twoL4removeAlgae.trajectory("Left2L4Algae2L3/2")
    val scoreMiddleA = twoL4removeAlgae.trajectory("Left2L4Algae2L3/3")
    val pickupLeft = twoL4removeAlgae.trajectory("Left2L4Algae2L3/6")
    val scoreRightB = twoL4removeAlgae.trajectory("Left2L4Algae2L3/7")
    val pickupRight = twoL4removeAlgae.trajectory("Left2L4Algae2L3/4")
    val scoreLeftA = twoL4removeAlgae.trajectory("Left2L4Algae2L3/5")

    twoL4removeAlgae.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).alongWith(
          robot.intake.holdCoral(),
          scorePreloadB.cmd()
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        pickupMiddle.cmd().alongWith(PremoveIntake(robot)) // replace with new ground intake command
      )
    )

    pickupMiddle.done().onTrue(
      Commands.sequence(
        Intake(robot),
        scoreMiddleA.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
        PremoveIntake(robot).alongWith(
          pickupLeft.cmd()
        )
      )
    )


    pickupLeft.done().onTrue(
      Commands.sequence(
        Intake(robot),
        scoreRightB.cmd()
          .alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    scoreRightB.done().onTrue(
      Commands.sequence(
        ScoreL3(robot, FieldConstants.ReefSide.RIGHT),
        PremoveIntake(robot).alongWith(
          pickupRight.cmd() // replace with new ground intake command
        )
      )
    )

    pickupRight.done().onTrue(
      Commands.sequence(
        Intake(robot),
        scoreLeftA.cmd()
          .alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    scoreLeftA.done().onTrue(
      Commands.sequence(
        ScoreL3(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return twoL4removeAlgae
  }

  // Elevator is cooked!
  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("right 4l4 Ground", this::rightGround4L4)
    autoChooser.addRoutine("left 2l4 +algae+ 2l3 ", this::rightCoralAlgae)
    autoChooser.addRoutine("RightTaxi", this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi", this::leftTaxi)
    autoChooser.addRoutine("l2 G", this::l2reefG) // middle l2
    autoChooser.addRoutine("l4 E & l4 D ", this::reefED) // 2 piece l4
    autoChooser.addRoutine("l4 E & l4 D + half ", this::reefEDhalf) // 2 piece l4
    autoChooser.addRoutine("l4 J & l4 L", this::reefJL) // 2 piece l4
    autoChooser.addRoutine("l4 J & l4 L + half", this::reefJLhalf) // 2 piece l4
    autoChooser.addRoutine("l4 J,K,L", this::reefJKL) // 3 piece l4
    autoChooser.addRoutine("l4 E,D,C", this::reefEDC) // 3 piece l4
    autoChooser.addRoutine("The Goat", this::americanRoutine) // america
    autoChooser.addRoutine("testing", this::middleRoutine)
    autoChooser.addRoutine("jwoj", this::l2Routine)
    autoChooser.addRoutine("Left Goat", this::LeftamericanRoutine)
  }

  fun ScoreL4(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestL4()
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoral())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun ScoreL3(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoral())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun Intake(robot: Robot): Command {
    return InstantCommand(robot.drive::stop)
      .andThen(robot.intake.intakeCoral())
      .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE))
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(robot.intake.holdCoral())
  }

  fun PremoveIntake(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
      .alongWith(robot.intake.intakeCoral())
  }
}
