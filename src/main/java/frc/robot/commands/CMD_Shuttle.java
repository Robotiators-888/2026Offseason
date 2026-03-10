package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shuttle extends RunCommand{
    private SUB_Index index;
    private SUB_Shooter shooter;
    private SUB_Intake intake;
    private SUB_PhotonVision photonVision;
    private CommandSwerveDrivetrain drivetrain;
    private final PIDController robotAngleController = new PIDController(3, 0, 0);
    private Pose2d targetPose = new Pose2d();
    private boolean isThetaErrorCorrect;
    public CMD_Shuttle (CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Index index, SUB_Shooter shooter, SUB_Intake intake) {
        super(()->{});
        this.index = index;
        this.shooter = shooter;
        this.intake = intake;
        this.photonVision = photonVision;
        this.drivetrain = drivetrain;
        robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
        isThetaErrorCorrect = false;
        addRequirements(photonVision, drivetrain, index, shooter, intake);
    }

    @Override
    public void initialize () {
    }

    @Override
    public void execute () {
        Pose2d currentPose = drivetrain.getPose();
        // Set targetPose to be correct
        Translation2d targetTranslation = targetPose.getTranslation();
        Rotation2d targetRotation = new Rotation2d(
            targetTranslation.getX() - currentPose.getX(),
            targetTranslation.getY() - currentPose.getY()
        );
        double omegaSpeed = robotAngleController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );
        double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
        isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(14) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 5;
        double distance = drivetrain.getPose().getTranslation().getDistance(
            photonVision.at_field.getTagPose(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
            ).map(pose -> pose.toPose2d().getTranslation().plus(
                    (currentPose.getY() >= 4) ? new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 90 : -90), 6) : new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 90 : -90), 2)
            )).orElse(drivetrain.getPose().getTranslation())
        );
        shooter.shootMeters(distance);
        index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM); // Keep metering wheel spinning
        boolean isShooterReady = shooter.atDesiredRPM();
        boolean isMeteringReady = Math.abs(index.intakeMeteringRPM() - Constants.Index.kINDEX_METERING_MOTOR_RPM) < 100;
        if (isThetaErrorCorrect && isShooterReady && isMeteringReady) {
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
        }
    }
}
