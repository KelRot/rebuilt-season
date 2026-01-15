// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;


public class HubAllign extends Command{
    private final Drive drive;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private PIDController pid;
    private double gyroRadian;
    private double targetRad;
    private Pose2d hubPose;
    private Pose2d botPose;

    public HubAllign(Drive drive) {
        this.drive = drive;
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

