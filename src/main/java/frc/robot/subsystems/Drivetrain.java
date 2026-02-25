// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  Pigeon2 gyro;
  private final SwerveModule[] dt;
  SwerveDrivePoseEstimator pose_estimator;
  Field2d field;
  CANBusStatus busUtilization;
  RobotConfig config;
  PPHolonomicDriveController controller;
  PIDConstants drive_constants;
  PIDConstants turn_constants;

  Rotation2d yaw_offset = Rotation2d.fromDegrees(90);

  NetworkTableInstance instance;
  NetworkTable table;

  DoubleSubscriber TranslationkP, TranslationkI, TranslationkD;
  DoubleSubscriber RotationkP, RotationkI, RotationkD;

  ChassisSpeeds current_speed;

  public Drivetrain() {
    gyro = new Pigeon2(0);

    set_gyro(0);
    this.dt = new SwerveModule[] {
        new SwerveModule(0, Constants.dt.mod0.drive_id, Constants.dt.mod0.turn_id, Constants.dt.mod0.can_coder,
            Constants.dt.mod0.turn_offset),
        new SwerveModule(1, Constants.dt.mod1.drive_id, Constants.dt.mod1.turn_id, Constants.dt.mod1.can_coder,
            Constants.dt.mod1.turn_offset),
        new SwerveModule(2, Constants.dt.mod2.drive_id, Constants.dt.mod2.turn_id, Constants.dt.mod2.can_coder,
            Constants.dt.mod2.turn_offset),
        new SwerveModule(3, Constants.dt.mod3.drive_id, Constants.dt.mod3.turn_id, Constants.dt.mod3.can_coder,
            Constants.dt.mod3.turn_offset)
    };
    reset_encoders();

    current_speed = Constants.dt.swerve_map.toChassisSpeeds(getModuleState());

    // NetworkTable
    instance = NetworkTableInstance.getDefault();

    table = instance.getTable("Pathplanner Constants");

    table.getDoubleTopic("TranslationkP").publish().setDefault(Constants.PathPlanner.translation_kP);
    table.getDoubleTopic("TranslationkI").publish().setDefault(Constants.PathPlanner.translation_kI);
    table.getDoubleTopic("TranslationkD").publish().setDefault(Constants.PathPlanner.translation_kD);

    table.getDoubleTopic("RotationkP").publish().setDefault(Constants.PathPlanner.rotation_kP);
    table.getDoubleTopic("RotationkI").publish().setDefault(Constants.PathPlanner.rotation_kI);
    table.getDoubleTopic("RotationkD").publish().setDefault(Constants.PathPlanner.rotation_kD);

    TranslationkP = table.getDoubleTopic("TranslationkP").subscribe(Constants.PathPlanner.translation_kP);
    TranslationkI = table.getDoubleTopic("TranslationkI").subscribe(Constants.PathPlanner.translation_kI);
    TranslationkD = table.getDoubleTopic("TranslationkD").subscribe(Constants.PathPlanner.translation_kD);

    RotationkP = table.getDoubleTopic("RotationkP").subscribe(Constants.PathPlanner.rotation_kP);
    RotationkI = table.getDoubleTopic("RotationkI").subscribe(Constants.PathPlanner.rotation_kI);
    RotationkD = table.getDoubleTopic("RotationkD").subscribe(Constants.PathPlanner.rotation_kD);

    // pose estimator to estimate where the robot is on the field using encoder and
    // yaw data
    pose_estimator = new SwerveDrivePoseEstimator(Constants.dt.swerve_map, get_yaw(),
        new SwerveModulePosition[] {
            this.dt[0].get_position(),
            this.dt[1].get_position(),
            this.dt[2].get_position(),
            this.dt[3].get_position()
        },
        new Pose2d());

    // field to visualize where pose estimator thinks robot is
    field = new Field2d();

    //controller = refreshController();
    drive_constants = new PIDConstants(Constants.PathPlanner.translation_kP, Constants.PathPlanner.translation_kI, Constants.PathPlanner.translation_kD);
    turn_constants = new PIDConstants(Constants.PathPlanner.rotation_kP, Constants.PathPlanner.rotation_kI, Constants.PathPlanner.rotation_kD);

    controller = new PPHolonomicDriveController(turn_constants, drive_constants);

    try {
      config = RobotConfig.fromGUISettings();
      // If Failed will return an error
    } catch (Exception exception) {
      exception.printStackTrace();
    }

    // Configures the AutoBuilder
    AutoBuilder.configure(
        this::get_Pose,
        this::reset_pose,
        this::get_current_speeds,
        (speed, feedforward) -> DriveRobotRelative(speed),
        controller,
        config,
        () -> flipTeam(),
        this);
  }

  public void set_gyro(double yaw) {
    // sets the gyros yaw to the yaw paramter
    // this method is in degrees
    gyro.setYaw(yaw);
  }

  // resets the encoders of each module to the cancoder offset
  public void reset_encoders() {
    for (SwerveModule module : this.dt) {
      module.reset_encoder();
    }
  }

  // returns the yaw of robot
  public Rotation2d get_yaw() {
    // gets the yaw as a Rotation 2d variable using Phoenix 6 API
    return gyro.getRotation2d().plus(yaw_offset);
  }

  // drive funciton
  public void drive(Translation2d translation, double rotation, boolean is_field_relative) {
    // creates swerve module states form joystick values which are in translation
    // and rotation
    SwerveModuleState[] swerve_module_states = Constants.dt.swerve_map.toSwerveModuleStates(is_field_relative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, get_yaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    // reduces all the swervemodule state speeds if one of them is over the max
    // speed to make it so that none are over but all are still the same ratio
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve_module_states, Constants.dt.max_speed);

    // sets the module states
    for (SwerveModule module : this.dt) {
      module.set_desired_state(swerve_module_states[module.module_number]);
    }
  }

  // Pathplanner Code

  // Function that returns a Estimated Position as a Pose2D variable from the Pose
  // Estimator
  public Pose2d get_Pose() {
    return this.pose_estimator.getEstimatedPosition();
  }

  // Resets the pose of the Robot
  // Uses current rotation, Module positions, and the current pose
  public void reset_pose(Pose2d pose) {
    // this.pose_estimator.resetPose(pose);
    this.pose_estimator.resetPosition(get_yaw(), swerveModulePositions(), pose);
  }

  // Returns the RelativeSpeed of the Robot using the Swerve_Kinematic to access
  // the module state
  public ChassisSpeeds get_current_speeds() {
    ChassisSpeeds speed = new ChassisSpeeds(
        current_speed.vxMetersPerSecond,
        current_speed.vyMetersPerSecond,
        current_speed.omegaRadiansPerSecond);
    return speed;
  }

  // Flips the path's for different team colors
  public boolean flipTeam() {
    // gets the alliance color from the driverstation
    var alliance = DriverStation.getAlliance();
    // if the alliance variable is present ...
    if (alliance.isPresent()) {
      // returns true
      // red alliance is true
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      // returns false
      // Blue alliance is false
      return false;
    }
  }

  // Pathplanner Drive
  // Uses speed(ChassisSpeed) to rotate the motors
  public void DriveRobotRelative(ChassisSpeeds speed) {
    SwerveModuleState[] ModuleStates = Constants.dt.swerve_map.toSwerveModuleStates(speed);
    SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, Constants.dt.max_speed);
    for (SwerveModule module : this.dt) {
      // SmartDashboard.putNumber("Module " + module.module_number + " v",
      // ModuleStates[module.module_number].speedMetersPerSecond);
      // SmartDashboard.putNumber("Module " + module.module_number + " w",
      // ModuleStates[module.module_number].angle.getRadians());
      module.set_desired_state(ModuleStates[module.module_number]);
    }
  }

  // Returns the SwerveModulePositions
  public SwerveModulePosition[] swerveModulePositions() {
    return new SwerveModulePosition[] {
        this.dt[0].get_position(),
        this.dt[1].get_position(),
        this.dt[2].get_position(),
        this.dt[3].get_position()
    };
  }

  // Returns the SwerveModuleState
  public SwerveModuleState[] getModuleState() {
    return new SwerveModuleState[] {
        this.dt[0].get_state(),
        this.dt[1].get_state(),
        this.dt[2].get_state(),
        this.dt[3].get_state()
    };
  }

  /*public PPHolonomicDriveController refreshController() {
    PIDConstants translationConstants = new PIDConstants(TranslationkP.get(), TranslationkI.get(), TranslationkD.get());
    PIDConstants rotationConstants = new PIDConstants(RotationkP.get(), RotationkI.get(), RotationkD.get());
    return new PPHolonomicDriveController(translationConstants, rotationConstants);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // the order of the modules is front left front right back left then back right
    // because that is the order the swerve map kinematics is defined in
    pose_estimator.update(get_yaw(), new SwerveModulePosition[] { this.dt[0].get_position(), this.dt[1].get_position(),
        this.dt[2].get_position(), this.dt[3].get_position() });
    // puts the robot position on the robot field and then puts the field on
    // smartdashboard
    field.setRobotPose(pose_estimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Gyro Yaw", get_yaw().minus(yaw_offset).getDegrees());

    current_speed = Constants.dt.swerve_map.toChassisSpeeds(getModuleState());

    SmartDashboard.putNumber("VX", current_speed.vxMetersPerSecond);
    SmartDashboard.putNumber("VY", current_speed.vyMetersPerSecond);
    SmartDashboard.putNumber("Radians", current_speed.omegaRadiansPerSecond);

    //controller = refreshController();

    // AutoBuilder.configure(
    //     this::get_Pose,
    //     this::reset_pose,
    //     this::get_current_speeds,
    //     (speed, feedforward) -> DriveRobotRelative(speed),
    //     controller,
    //     config,
    //     () -> flipTeam(),
    //     this);
    
    // for (SwerveModule mod : dt) {
    //  mod.periodic();
    // }
    // dt[0].periodic();
    // dt[1].periodic();
    // dt[2].periodic();
    // dt[3].periodic();
  }
}
