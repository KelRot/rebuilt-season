// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class HubAllign extends Command{
    private final Drive drive;
    private final GyroIOPigeon2 gyro;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private PIDController pid;
    private double gyroRadian;
    private double targetRad;
    private Pose2d hubPose;
    private Pose2d botPose;

    public HubAllign(Drive drive, GyroIOPigeon2 gyro) {
        this.drive = drive;
        this.gyro = gyro;
        gyroRadian = drive.getPose().getRotation().getRadians();
        pid = new PIDController(ANGLE_KP, 0.0, ANGLE_KD);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        hubPose = Constants.DriveConstants.hub;
        botPose = this.drive.getPose();
        targetRad = Math.atan2(hubPose.getY() - botPose.getY(), hubPose.getX() - botPose.getX());

        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.runVelocity(new ChassisSpeeds (0, 0, pid.calculate(gyroRadian, targetRad)));
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

