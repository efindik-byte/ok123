// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static final int kShooterMotorCanId = 50; // Kraken X60 (Talon FX)
    private static final String kShooterCanBus = "rio";
    private static final int kFeederMotorCanId = 20;
    private static final int kIntakeMotorCanId = 21; // NEO brushless
    private static final int kIntakeArmMotorCanId = 22; // NEO brushless

    private static final double kIntakePercentOutput = 0.8;
    private static final double kFeederPercentOutput = -0.6;
    private static final double kFeederReversePercentOutput = 0.6;
    private static final double kShooterPercentOutput = 1.0;
    private static final double kShooterBoostPercentOutput = 1.0; // Hold D-pad Right while shooter is on
    private static final double kShooterStartPercentOutput = 0.05;
    private static final double kShooterRampUpDurationSec = 1.0;
    private static final double kShooterRampDownDurationSec = 1.0;
    private static final double kShooterRampSettledFraction = 0.99;
    private static final double kIntakeArmUpPercentOutput = -0.3;   // counterclockwise
    private static final double kIntakeArmDownPercentOutput = 0.3;  // clockwise
    private static final double kIntakeDiagPeriodSec = 0.25;
    private static final String kLimelightTableName = "limelight";
    private static final double kLimelightTurnKp = 2.0; // rad/s per rad of tx error
    private static final double kLimelightMaxTurnRateRadPerSec = 1.5;
    private static final double kLimelightTxDeadbandDeg = 1.0;
    private static final double kInputDeadband = 0.10;
    private static final double kOperatorDriveSpeedScale = 0.5; // Editable operator drive/turn speed scale
    private static final double kIntakeArmPosLogPeriodSec = 0.5;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final TalonFX shooterMotor = new TalonFX(kShooterMotorCanId, kShooterCanBus);
    private final SparkMax feederMotor = new SparkMax(kFeederMotorCanId, MotorType.kBrushed);
    private final SparkMax intakeMotor = new SparkMax(kIntakeMotorCanId, MotorType.kBrushless);
    private final SparkMax intakeArmMotor = new SparkMax(kIntakeArmMotorCanId, MotorType.kBrushless);
    private final DutyCycleOut shooterRequest = new DutyCycleOut(0);
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(kLimelightTableName);
    private double lastIntakeDiagTimeSec = -1.0;
    private double lastIntakeArmPosLogTimeSec = -1.0;
    private double shooterAppliedPercent = 0.0;
    private double lastShooterUpdateTimeSec = Timer.getFPGATimestamp();
    private boolean limelightAlignModeLast = false;
    private boolean limelightHadTagLast = false;
    private double lastLimelightDebugTimeSec = -1.0;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-getRequestedLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-getRequestedLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(getRequestedRotationalRate()) // X held: auto-align to tag, else manual right stick
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Mechanisms (same controls on driver and operator; operator has priority):
        // - Hold LT: intake
        // - Hold LB: feeder reverse
        // - Hold RB: feeder
        // - Hold RT: shooter
        // - Hold D-pad Right with shooter ON: shooter boost speed
        final Trigger intakeTrigger = new Trigger(this::isIntakeRequested);
        final Trigger feederReverseTrigger = new Trigger(this::isFeederReverseRequested);
        final Trigger feederTrigger = new Trigger(this::isFeederRequested);
        final Trigger shooterTrigger = new Trigger(this::isShooterRequested);
        final Trigger intakeArmUpTrigger = new Trigger(this::isIntakeArmUpRequested);
        final Trigger intakeArmDownTrigger = new Trigger(this::isIntakeArmDownRequested);

        intakeTrigger.whileTrue(Commands.runEnd(
            () -> {
                intakeMotor.set(kIntakePercentOutput);
                logIntakeDiagnostics();
            },
            () -> intakeMotor.set(0.0)
        ));
        intakeTrigger.onTrue(Commands.runOnce(() -> {
            lastIntakeDiagTimeSec = -1.0;
            logControl("LT detected -> Intake ON");
        }));
        intakeTrigger.onFalse(Commands.runOnce(() -> logControl("LT released -> Intake OFF")));

        feederTrigger.whileTrue(Commands.runEnd(
            () -> feederMotor.set(kFeederPercentOutput),
            () -> feederMotor.set(0.0)
        ));
        feederTrigger.onTrue(Commands.runOnce(() -> logControl("RB detected -> Feeder ON")));
        feederTrigger.onFalse(Commands.runOnce(() -> logControl("RB released -> Feeder OFF")));

        feederReverseTrigger.whileTrue(Commands.runEnd(
            () -> feederMotor.set(kFeederReversePercentOutput),
            () -> feederMotor.set(0.0)
        ));
        feederReverseTrigger.onTrue(Commands.runOnce(() -> logControl("LB detected -> Feeder REVERSE ON")));
        feederReverseTrigger.onFalse(Commands.runOnce(() -> logControl("LB released -> Feeder REVERSE OFF")));

        // Shooter always runs a smooth exponential profile between stop and full power.
        // RT chooses the full-power target; releasing RT returns smoothly to zero.
        RobotModeTriggers.teleop().whileTrue(Commands.run(() -> {
            final double targetPercent = shooterTrigger.getAsBoolean()
                ? (isShooterBoostRequested() ? kShooterBoostPercentOutput : kShooterPercentOutput)
                : 0.0;
            updateShooterOutput(targetPercent);
        }));
        shooterTrigger.onTrue(Commands.runOnce(() -> logControl("RT detected -> Shooter ON")));
        shooterTrigger.onFalse(Commands.runOnce(() -> logControl("RT released -> Shooter OFF")));

        intakeArmUpTrigger.whileTrue(Commands.runEnd(
            () -> intakeArmMotor.set(kIntakeArmUpPercentOutput),
            () -> intakeArmMotor.set(0.0)
        ));
        intakeArmUpTrigger.onTrue(Commands.runOnce(() -> logControl("D-pad Up detected -> Intake Arm CCW")));
        intakeArmUpTrigger.onFalse(Commands.runOnce(() -> logControl("D-pad Up released -> Intake Arm OFF")));

        intakeArmDownTrigger.whileTrue(Commands.runEnd(
            () -> intakeArmMotor.set(kIntakeArmDownPercentOutput),
            () -> intakeArmMotor.set(0.0)
        ));
        intakeArmDownTrigger.onTrue(Commands.runOnce(() -> logControl("D-pad Down detected -> Intake Arm CW")));
        intakeArmDownTrigger.onFalse(Commands.runOnce(() -> logControl("D-pad Down released -> Intake Arm OFF")));

        // Reset field-centric heading on left stick press.
        new Trigger(this::isReseedRequested).onTrue(Commands.sequence(
            Commands.runOnce(() -> logControl("Left stick pressed -> Reseed field-centric heading")),
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        ));

        // Debug intake arm position at 500ms cadence in teleop.
        RobotModeTriggers.teleop().whileTrue(Commands.run(this::maybeLogIntakeArmPosition));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    private void logControl(String message) {
        DriverStation.reportWarning("[CONTROLS] " + message, false);
    }

    private void logIntakeDiagnostics() {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastIntakeDiagTimeSec > 0.0 && (nowSec - lastIntakeDiagTimeSec) < kIntakeDiagPeriodSec) {
            return;
        }
        lastIntakeDiagTimeSec = nowSec;

        DriverStation.reportWarning(String.format(
            "[INTAKE] cmd=%.2f applied=%.2f current=%.2fA bus=%.2fV temp=%.1fC err=%s",
            kIntakePercentOutput,
            intakeMotor.getAppliedOutput(),
            intakeMotor.getOutputCurrent(),
            intakeMotor.getBusVoltage(),
            intakeMotor.getMotorTemperature(),
            intakeMotor.getLastError()
        ), false);
    }

    private void updateShooterOutput(double targetPercent) {
        final double nowSec = Timer.getFPGATimestamp();
        final double dtSec = Math.max(0.0, nowSec - lastShooterUpdateTimeSec);
        lastShooterUpdateTimeSec = nowSec;

        final double rampDurationSec = targetPercent > shooterAppliedPercent
            ? kShooterRampUpDurationSec
            : kShooterRampDownDurationSec;
        final double tauSec = -rampDurationSec / Math.log(1.0 - kShooterRampSettledFraction);
        final double alpha = 1.0 - Math.exp(-dtSec / tauSec);

        if (targetPercent > 0.0 && shooterAppliedPercent <= 0.0) {
            shooterAppliedPercent = Math.min(kShooterStartPercentOutput, targetPercent);
        }

        shooterAppliedPercent += (targetPercent - shooterAppliedPercent) * alpha;
        shooterAppliedPercent = Math.max(0.0, Math.min(1.0, shooterAppliedPercent));

        shooterMotor.setControl(shooterRequest.withOutput(shooterAppliedPercent));
    }

    private double getRequestedRotationalRate() {
        // Hold X to auto-align to a visible AprilTag using Limelight tx.
        final boolean alignModeActive = isAlignRequested();
        if (alignModeActive != limelightAlignModeLast) {
            DriverStation.reportWarning(
                alignModeActive ? "[LIMELIGHT] X held -> align mode ON" : "[LIMELIGHT] X released -> align mode OFF",
                false
            );
            limelightAlignModeLast = alignModeActive;
        }

        if (alignModeActive) {
            final double tv = limelightTable.getEntry("tv").getDouble(0.0);
            if (tv >= 1.0) {
                final double txDeg = limelightTable.getEntry("tx").getDouble(0.0);
                if (!limelightHadTagLast) {
                    DriverStation.reportWarning("[LIMELIGHT] Tag detected", false);
                    limelightHadTagLast = true;
                }

                if (Math.abs(txDeg) < kLimelightTxDeadbandDeg) {
                    maybeLogLimelightDebug(txDeg, 0.0);
                    return 0.0;
                }
                final double txRad = Math.toRadians(txDeg);
                final double rawTurnRate = -kLimelightTurnKp * txRad;
                final double turnRate = Math.max(
                    -kLimelightMaxTurnRateRadPerSec,
                    Math.min(kLimelightMaxTurnRateRadPerSec, rawTurnRate)
                );
                maybeLogLimelightDebug(txDeg, turnRate);
                return turnRate;
            }
            if (limelightHadTagLast) {
                DriverStation.reportWarning("[LIMELIGHT] No tag detected", false);
                limelightHadTagLast = false;
            }
            // No tag -> no auto-turn.
            return 0.0;
        }

        limelightHadTagLast = false;
        // Normal driver rotation when X is not held.
        return -getRequestedRightX() * MaxAngularRate;
    }

    private double getRequestedLeftY() {
        return getPrioritizedAxis(driverController.getLeftY(), operatorController.getLeftY());
    }

    private double getRequestedLeftX() {
        return getPrioritizedAxis(driverController.getLeftX(), operatorController.getLeftX());
    }

    private double getRequestedRightX() {
        return getPrioritizedAxis(driverController.getRightX(), operatorController.getRightX());
    }

    private double getPrioritizedAxis(double driverAxis, double operatorAxis) {
        if (isOperatorControllingRobot()) {
            return operatorAxis * kOperatorDriveSpeedScale;
        }
        return driverAxis;
    }

    private boolean isOperatorControllingRobot() {
        return Math.abs(operatorController.getLeftX()) > kInputDeadband
            || Math.abs(operatorController.getLeftY()) > kInputDeadband
            || Math.abs(operatorController.getRightX()) > kInputDeadband
            || operatorController.getLeftTriggerAxis() > 0.2
            || operatorController.getRightTriggerAxis() > 0.2
            || operatorController.leftBumper().getAsBoolean()
            || operatorController.rightBumper().getAsBoolean()
            || operatorController.povUp().getAsBoolean()
            || operatorController.povDown().getAsBoolean()
            || operatorController.povRight().getAsBoolean()
            || operatorController.leftStick().getAsBoolean()
            || operatorController.x().getAsBoolean();
    }

    private boolean isInputFromOperator(boolean driverInput, boolean operatorInput) {
        return isOperatorControllingRobot() ? operatorInput : driverInput;
    }

    private boolean isIntakeRequested() {
        return isInputFromOperator(
            driverController.getLeftTriggerAxis() > 0.2,
            operatorController.getLeftTriggerAxis() > 0.2
        );
    }

    private boolean isFeederRequested() {
        return isInputFromOperator(
            driverController.rightBumper().getAsBoolean(),
            operatorController.rightBumper().getAsBoolean()
        );
    }

    private boolean isFeederReverseRequested() {
        return isInputFromOperator(
            driverController.leftBumper().getAsBoolean(),
            operatorController.leftBumper().getAsBoolean()
        );
    }

    private boolean isShooterRequested() {
        return isInputFromOperator(
            driverController.getRightTriggerAxis() > 0.2,
            operatorController.getRightTriggerAxis() > 0.2
        );
    }

    private boolean isShooterBoostRequested() {
        return isInputFromOperator(
            driverController.povRight().getAsBoolean(),
            operatorController.povRight().getAsBoolean()
        );
    }

    private boolean isIntakeArmUpRequested() {
        return isInputFromOperator(
            driverController.povUp().getAsBoolean(),
            operatorController.povUp().getAsBoolean()
        );
    }

    private boolean isIntakeArmDownRequested() {
        return isInputFromOperator(
            driverController.povDown().getAsBoolean(),
            operatorController.povDown().getAsBoolean()
        );
    }

    private boolean isAlignRequested() {
        return isInputFromOperator(
            driverController.x().getAsBoolean(),
            operatorController.x().getAsBoolean()
        );
    }

    private boolean isReseedRequested() {
        return isInputFromOperator(
            driverController.leftStick().getAsBoolean(),
            operatorController.leftStick().getAsBoolean()
        );
    }

    private void maybeLogIntakeArmPosition() {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastIntakeArmPosLogTimeSec > 0.0 && (nowSec - lastIntakeArmPosLogTimeSec) < kIntakeArmPosLogPeriodSec) {
            return;
        }
        lastIntakeArmPosLogTimeSec = nowSec;

        final double rotations = intakeArmMotor.getEncoder().getPosition();
        final double degrees = rotations * 360.0;
        DriverStation.reportWarning(
            String.format("[INTAKE_ARM] pos=%.3f rotations (%.1f deg)", rotations, degrees),
            false
        );
    }

    private void maybeLogLimelightDebug(double txDeg, double turnRateRadPerSec) {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastLimelightDebugTimeSec > 0.0 && (nowSec - lastLimelightDebugTimeSec) < 0.25) {
            return;
        }
        lastLimelightDebugTimeSec = nowSec;
        DriverStation.reportWarning(
            String.format("[LIMELIGHT] tx=%.2f deg, autoTurn=%.2f rad/s", txDeg, turnRateRadPerSec),
            false
        );
    }
}
