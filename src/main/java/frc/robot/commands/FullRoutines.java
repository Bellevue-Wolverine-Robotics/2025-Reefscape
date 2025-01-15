// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class FullRoutines {

  private FullRoutines() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  // public static Command getShootAtSpeakerRoutine(FlyWheelSubsystem flywheelSubsystem) {
  //   return new SequentialCommandGroup(
  //       //new FlywheelAimSpeakerCommand(flywheelSubsystem),

  //       new SequentialCommandGroup(
  //         new InstantCommand(() -> {
  //           flywheelSubsystem.setShoulderAimSpeaker();
  //         }),
  //         new WaitCommand(0.8),
  //         new InstantCommand(() -> {
  //           flywheelSubsystem.setElbowAimSpeaker();
  //         })

  //       )


  //       //new InstantCommand(flywheelSubsystem::startShooter, flywheelSubsystem),

  //       //new WaitCommand(1),
  //       //new InstantCommand(flywheelSubsystem::startFeeder, flywheelSubsystem),
  //       //new WaitCommand(3),
  //       //new InstantCommand(flywheelSubsystem::stopShooter, flywheelSubsystem),
  //       //new InstantCommand(flywheelSubsystem::stopFeeder, flywheelSubsystem),
  //       //new FlywheelMoveToMakeSpaceForIntakeCommand(flywheelSubsystem)
  //     );
  // }

  // public static Command getFullIntakeRoutine(IntakeSubsystem intakeSubsystem, FlyWheelSubsystem flywheelSubsystem) {
  //   return new SequentialCommandGroup(
  //       new FlywheelMoveToMakeSpaceForIntakeCommand(flywheelSubsystem),
  //       // new DeployIntakeCommand(intakeSubsystem),
  //       // new StowIntakeCommand(intakeSubsystem),
  //       new FlywheelAimIntakeReceiveCommand(flywheelSubsystem)
  
        
  //       );
  // }

  // public static Command prepareToClimb(IntakeSubsystem intakeSubsystem) {
  //   return new SequentialCommandGroup(
  //       new DeployIntakeCommandWithoutStartingMotor(intakeSubsystem));
  // }
}
// new ParallelCommandGroup(
// new StartIntakeCommand(intakeSubsystem),
// /*
// * AFTER INTAKE IS IN RECIEVING POSITION,
// * INTAKE WHEELS STARTS SPINNING
// */
// new FlywheelAimIntakeReceiveCommand(flywheelSubsystem)

// ),
// /*
// * AT THIS POINT INTAKE IS IN RECIEVING POSITION AND THE INTAKE IS SPINNNING
// * THE FLYWHEEL IS IN POSITION TO RECIEVE FROM INTAKE
// */
// new GetIntakeStatus(intakeSubsystem),
// /*
// * AT THIS POINT ARM HAS ACQUIRED NOTE(VERIFIED BY LIMIT SWITCH)
// */
// new StopIntakeCommand(intakeSubsystem),
// /*
// * AT THIS POINT THE INTAKE ARM IS AT REST POSITION AND IN POSITION FOR
// FLYWHEEL
// * INTAKE TO RECIEVE
// * INTAKE MOTOR WILL STOP BECAUSE IT HAS ACUIRED NOTE
// */

// // TODO: TURN THIS INTO A COMMAND THAT SPINS FOR LIKE HALF A SECOND
// new InstantCommand(
// () -> {
// flywheelSubsystem.startFeeder();
// }
// /*
// * FLYWHEEL IS ALREAY IN POSITION, FLYWHEEL DOING INTAKE
// */
// )
