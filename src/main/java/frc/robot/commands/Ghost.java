package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class Ghost extends Command {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    PIDController x, y;
    ProfiledPIDController theta;
    // Pose3d[] blueReefTags = new Pose3d[] { layout.getTagPose(17).get(),
    // layout.getTagPose(18).get(), layout.getTagPose(19).get(),
    // layout.getTagPose(20).get(), layout.getTagPose(21).get(),
    // layout.getTagPose(22).get() };
    // Pose3d[] redReefTags = new Pose3d[] { layout.getTagPose(6).get(),
    // layout.getTagPose(7).get(), layout.getTagPose(8).get(),
    // layout.getTagPose(9).get(), layout.getTagPose(10).get(),
    // layout.getTagPose(11).get() };
    Drive drive;
    Vision vision;
    boolean isFinished;

    /** Creates a new AutoAlignTest. */
    public Ghost(Drive drive, Vision vision) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drive = drive;
        this.vision = vision;
        this.isFinished = false;

        this.x = new PIDController(5, 0, 0);
        this.y = new PIDController(5, 0, 0);
        this.theta = new ProfiledPIDController(1, 0, 0, new Constraints(Math.PI, Math.PI * Math.PI));

        addRequirements(drive, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.isFinished = false;

        // drive.setPose(new Pose2d(8, 8, new Rotation2d()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();

        int targetId = 21;
        Rotation2d rotation2d = layout.getTagPose(targetId).get().getRotation().toRotation2d();
        Pose3d tagPose = layout.getTagPose(targetId).get();

        double r = rotation2d.getRadians();

        double length = Units.Inch.of(10).in(Meters);
        double new_x = length * Math.cos(r);
        double new_y = length * Math.sin(r);

        // 0.403225
        Pose2d targetPose2d = new Pose2d(tagPose.getX() + new_x, tagPose.getY() + new_y, rotation2d);

        double xeffort = x.calculate(robotPose.getX(), targetPose2d.getX());
        double yeffort = y.calculate(robotPose.getY(), targetPose2d.getY());
        double thetaeffort = theta.calculate(
                robotPose.getRotation().getRadians(), targetPose2d.getRotation().getRadians());

        ChassisSpeeds speeds = new ChassisSpeeds(xeffort, yeffort, thetaeffort);
        Pose2d error = robotPose.relativeTo(targetPose2d);

        // if (change.dx <= 0.5 && change.dy <= 0.5) {
        // isFinished = true;
        // }

        Logger.recordOutput("AutoAlign/target", targetPose2d);
        Logger.recordOutput("AutoAlign/target_tag", tagPose);
        Logger.recordOutput("AutoAlign/angle", rotation2d.getRadians());

        Logger.recordOutput("AutoAlign/rotation2d", rotation2d);

        Logger.recordOutput("AutoAlign/error_x", error.getX());
        Logger.recordOutput("AutoAlign/error_y", error.getY());

        drive.runVelocity(speeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
