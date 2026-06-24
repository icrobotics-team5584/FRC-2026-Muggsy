#include "subsystems/SubDrivebase.h"

#include "utilities/ICGeometry.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc/RobotBase.h>

SubDrivebase::SubDrivebase() {
  logger::Log("Drivebase/P2P/Rotation Controller", &_rotationP2pController);
  logger::Log("Drivebase/P2P/Translation Controller", &_translationP2pController);

  _rotationP2pController.EnableContinuousInput(0, 1);

  ctre::phoenix6::configs::Pigeon2Configuration gyroConfig;
  gyroConfig.MountPose.MountPosePitch = 0_deg;
  gyroConfig.MountPose.MountPoseRoll = 0_deg;
  gyroConfig.MountPose.MountPoseYaw = 0_deg;
  _gyro.GetConfigurator().Apply(gyroConfig);
}

void SubDrivebase::Periodic() {
  auto loopStart = frc::GetTime();

  LogDrivebaseStates();
  UpdateOdometry();

  logger::Log("Drivebase/Loop Time", (frc::GetTime() - loopStart));
}

void SubDrivebase::SimulationPeriodic() {
  _frontLeft.UpdateSim(20_ms);
  _frontRight.UpdateSim(20_ms);
  _backLeft.UpdateSim(20_ms);
  _backRight.UpdateSim(20_ms);

  auto rotSpeed = _kinematics
                    .ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                      _backLeft.GetState(), _backRight.GetState())
                    .omega;
  units::radian_t changeInRot = rotSpeed * 20_ms;
  units::degree_t newHeading = GetGyroAngle().RotateBy(changeInRot).Degrees();
  _gyro.SetYaw(newHeading);

  wpi::array<frc::SwerveModulePosition, 4U> states = {_frontLeft.GetPosition(),
    _frontRight.GetPosition(), _backLeft.GetPosition(), _backRight.GetPosition()};

  PoseHandler::GetInstance().AddSimOdometryMeasurement(
    GetGyroAngle(), states, true, PoseHandler::GetInstance().GetPose().Rotation());
}

// Commands

void SubDrivebase::LogDrivebaseStates() {
  logger::Log("Drivebase/GyroAngle/Roll", SubDrivebase::GetInstance().GetRoll());
  logger::Log("Drivebase/GyroAngle/Pitch", SubDrivebase::GetInstance().GetPitch());
  logger::Log("Drivebase/GyroAngle/ApproxTiltMagnitude", GetApproxTiltMagnitude());
  logger::Log("Drivebase/Coast Button", CheckCoastButton().Get());

  logger::Log("Drivebase/velocity", GetVelocity());
  logger::Log("Drivebase/velocity/desired field relative vx", GetDesiredChassisSpeeds().vx);
  logger::Log("Drivebase/velocity/desired field relative vy", GetDesiredChassisSpeeds().vy);
  logger::Log("Drivebase/velocity/desired angular velocity", GetDesiredAngularVelocity());

  logger::Log("Drivebase/velocity/field relative vx", GetChassisSpeeds().vx);
  logger::Log("Drivebase/velocity/field relative vy", GetChassisSpeeds().vy);

  logger::Log("Drivebase/Internal Encoder Swerve States",
    wpi::array{
      _frontLeft.GetState(), _frontRight.GetState(), _backLeft.GetState(), _backRight.GetState()});
  logger::Log("Drivebase/CANCoder Swerve States",
    wpi::array{_frontLeft.GetCANCoderState(), _frontRight.GetCANCoderState(),
      _backLeft.GetCANCoderState(), _backRight.GetCANCoderState()});
  logger::Log("Drivebase/Pigeon raw angle", _gyro.GetYaw().GetValue().value());
  logger::Log("Drivebase/Pigeon raw Rotation2d", _gyro.GetRotation2d().Degrees());

  logger::Log("Drivebase/DistanceDrivenRotations/fl", _frontLeft.GetDrivenRotations());
  logger::Log("Drivebase/DistanceDrivenRotations/fr", _frontRight.GetDrivenRotations());
  logger::Log("Drivebase/DistanceDrivenRotations/bl", _backLeft.GetDrivenRotations());
  logger::Log("Drivebase/DistanceDrivenRotations/br", _backRight.GetDrivenRotations());

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();
}

wpi::array<frc::SwerveModulePosition, 4U> SubDrivebase::GetSwerveStates() {
  return {_frontLeft.GetPosition(), _frontRight.GetPosition(), _backLeft.GetPosition(),
    _backRight.GetPosition()};
}

void SubDrivebase::UpdateOdometry() {
  wpi::array<frc::SwerveModulePosition, 4U> states = GetSwerveStates();

  if (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    PoseHandler::GetInstance().AddOdometryMeasurement(GetGyroAngle(), states);
  } else {
    PoseHandler::GetInstance().AddOdometryMeasurement(GetGyroAngle() - 180_deg, states);
  }
}

void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();

  _frontLeft.ConfigTurnMotor();
  _frontRight.ConfigTurnMotor();
  _backLeft.ConfigTurnMotor();
  _backRight.ConfigTurnMotor();
}

frc2::CommandPtr SubDrivebase::SyncSensor() {
  return RunOnce([this] { SyncSensors(); });
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.SetYaw(startingAngle);
}

frc2::CommandPtr SubDrivebase::ZeroRotation(const std::function<units::degree_t()>& startingAngle) {
  return RunOnce([this, startingAngle] {
    auto startingAngleVal = startingAngle();
    logger::Log("Drivebase/ZeroRotation/startingAngle", startingAngleVal);

    ResetGyroHeading(startingAngleVal);
    frc::Pose2d oldPose = PoseHandler::GetInstance().GetPose();

    frc::Pose2d newPose{oldPose.X(), oldPose.Y(), startingAngleVal};

    PoseHandler::GetInstance().SetPose(newPose, GetSwerveStates());
  });
}

void SubDrivebase::SetBrakeMode(bool mode) {
  _frontLeft.SetBreakMode(mode);
  _frontRight.SetBreakMode(mode);
  _backLeft.SetBreakMode(mode);
  _backRight.SetBreakMode(mode);
}

void SubDrivebase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
  units::turns_per_second_t rot, bool fieldRelative,
  std::optional<std::array<units::newton_t, 4>> xForceFeedforwards,
  std::optional<std::array<units::newton_t, 4>> yForceFeedforwards) {
  // Optionally convert speeds to field relative
  auto speeds = fieldRelative
                  ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetGyroAngle())
                  : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Discretize to get rid of translational drift while rotating
  speeds = frc::ChassisSpeeds::Discretize(speeds, 60_ms);

  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(speeds);

  // Set speed limit and apply speed limit to all modules
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
    &states, frc::SmartDashboard::GetNumber(
               "Drivebase/Config/Max Velocity", drivebaseConfig::MAX_VELOCITY.value()) *
               1_mps);

  // Extract force feedforwards
  std::array<units::newton_t, 4> defaults{0_N, 0_N, 0_N, 0_N};
  auto [flXForce, frXForce, blXForce, brXForce] = xForceFeedforwards.value_or(defaults);
  auto [flYForce, frYForce, blYForce, brYForce] = yForceFeedforwards.value_or(defaults);

  // Setting modules from aquired states
  logger::Log("Drivebase/Desired Swerve States", states);
  auto [fl, fr, bl, br] = states;
  _frontLeft.SetDesiredState(fl, flXForce, flYForce);
  _frontRight.SetDesiredState(fr, frXForce, frYForce);
  _backLeft.SetDesiredState(bl, blXForce, blYForce);
  _backRight.SetDesiredState(br, brXForce, brYForce);
}

frc2::CommandPtr SubDrivebase::Drive(
  const std::function<frc::ChassisSpeeds()>& speeds, bool fieldOriented) {
  return Run([this, speeds, fieldOriented] {
    auto speedVal = speeds();
    Drive(speedVal.vx, speedVal.vy, speedVal.omega, fieldOriented);
  }).FinallyDo([this] { Drive(0_mps, 0_mps, 0_deg_per_s, false); });
}

frc2::CommandPtr SubDrivebase::LockWheelsInXShape() {
  return Run([this] {
    auto fl = frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}};
    auto fr = frc::SwerveModuleState{0_mps, frc::Rotation2d{135_deg}};
    auto bl = frc::SwerveModuleState{0_mps, frc::Rotation2d{135_deg}};
    auto br = frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}};

    _frontLeft.SetDesiredState(fl);
    _frontRight.SetDesiredState(fr);
    _backLeft.SetDesiredState(bl);
    _backRight.SetDesiredState(br);
  });
}

frc2::CommandPtr SubDrivebase::DriveOverBump(
  frc::ChassisSpeeds fieldRelativeSpeeds, frc::Translation2d allianceRelativeEndXY) {
  return Drive([fieldRelativeSpeeds] { return fieldRelativeSpeeds; }, true)
    .WithDeadline(frc2::cmd::Sequence(
      frc2::cmd::RunOnce([] { logger::Log("Drivebase/DriveOverBump/State", 1); }),
      frc2::cmd::WaitUntil([this] {
        return (GetApproxTiltMagnitude() > 5_deg);  // ascending
      }),
      frc2::cmd::RunOnce([] { logger::Log("Drivebase/DriveOverBump/State", 2); }),
      frc2::cmd::WaitUntil([this] {
        return (GetApproxTiltMagnitude() < 5_deg);  // peak
      }),
      frc2::cmd::RunOnce([] { logger::Log("Drivebase/DriveOverBump/State", 3); }),
      frc2::cmd::WaitUntil([this] {
        return (GetApproxTiltMagnitude() >
                5_deg);  // descending. note that tilt magnitude is always positive
      }),
      frc2::cmd::RunOnce([] { logger::Log("Drivebase/DriveOverBump/State", 4); }),
      frc2::cmd::WaitUntil([this] {
        return (GetApproxTiltMagnitude() < 2_deg);  // done
      }),
      frc2::cmd::RunOnce([] { logger::Log("Drivebase/DriveOverBump/State", 5); })))
    .Unless([] { return frc::RobotBase::IsSimulation(); })
    .FinallyDo([this, allianceRelativeEndXY] {
      auto endPose =
        icGeometry::GetFieldRelativePose(frc::Pose2d(allianceRelativeEndXY, GetGyroAngle(false)));
      SetPose(endPose);
    });
}

// Getters & calculations
frc::Rotation2d SubDrivebase::GetGyroAngle(bool allianceRelative) {
  auto alliance = frc::DriverStation::GetAlliance();
  if (!allianceRelative || alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
                             frc::DriverStation::Alliance::kBlue) {
    return _gyro.GetRotation2d();
  }
  return _gyro.GetRotation2d() - 180_deg;
}

units::degree_t SubDrivebase::GetPitch() {
  return (_gyro.GetPitch().GetValue());
}

units::degree_t SubDrivebase::GetRoll() {
  return (_gyro.GetRoll().GetValue());
}

units::degree_t SubDrivebase::GetApproxTiltMagnitude() {
  // This is only accurate for small pitch and roll angles
  return units::math::hypot(GetPitch(), GetRoll());
}

units::meters_per_second_t SubDrivebase::GetVelocity() {
  auto speeds = _kinematics.ToChassisSpeeds(
    _frontLeft.GetState(), _frontRight.GetState(), _backLeft.GetState(), _backRight.GetState());
  return units::math::hypot(speeds.vx, speeds.vy);
}

frc::ChassisSpeeds SubDrivebase::GetChassisSpeeds(bool fieldRelative) {
  auto speeds = _kinematics.ToChassisSpeeds(
    _frontLeft.GetState(), _frontRight.GetState(), _backLeft.GetState(), _backRight.GetState());
  if (fieldRelative) {
    speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, GetGyroAngle(true).Degrees());
  }

  return speeds;
}

frc::ChassisSpeeds SubDrivebase::GetDesiredChassisSpeeds(bool fieldRelative) {
  auto speeds = _kinematics.ToChassisSpeeds(_frontLeft.GetDesiredState(),
    _frontRight.GetDesiredState(), _backLeft.GetDesiredState(), _backRight.GetDesiredState());
  if (fieldRelative) {
    speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(speeds, GetGyroAngle(false).Degrees());
  }

  return speeds;
}

units::degrees_per_second_t SubDrivebase::GetDesiredAngularVelocity() {
  auto speeds = _kinematics.ToChassisSpeeds(_frontLeft.GetDesiredState(),
    _frontRight.GetDesiredState(), _backLeft.GetDesiredState(), _backRight.GetDesiredState());
  return speeds.omega;
}

frc2::Trigger SubDrivebase::CheckCoastButton() {
  return frc2::Trigger{[this] { return !_toggleBrakeCoast.Get(); }};
}

units::turns_per_second_t SubDrivebase::CalcRotateSpeed(units::turn_t rotationError) {
  auto omega = _rotationP2pController.Calculate(rotationError.value(), 0) * 1_rad_per_s;
  return omega;
}

units::degree_t SubDrivebase::CalcAngleToTarget(frc::Translation2d target) {
  auto currentPose = PoseHandler::GetInstance().GetPose();
  frc::Translation2d robotToTarget = target - currentPose.Translation();
  return robotToTarget.Angle().Degrees();
}

frc::ChassisSpeeds SubDrivebase::CalcDriveToPoseSpeeds(frc::Pose2d targetPose) {
  // Find target and current values
  units::meter_t targetXMeters = targetPose.X();
  units::meter_t targetYMeters = targetPose.Y();
  units::turn_t targetRotation = targetPose.Rotation().Radians();

  frc::Pose2d currentPose = PoseHandler::GetInstance().GetPose();
  units::meter_t currentXMeters = currentPose.X();
  units::meter_t currentYMeters = currentPose.Y();
  units::turn_t currentRotation = frc::InputModulus(GetGyroAngle(true).Degrees(), 0_deg, 360_deg);

  // Create a vector between current position and target position
  frc::Translation2d translationVector =
    frc::Translation2d(targetXMeters - currentXMeters, targetYMeters - currentYMeters);

  // Use PID controllers to calculate speeds
  auto translationSpeed =
    _translationP2pController.Calculate(0, translationVector.Norm().value()) * 1_mps;
  auto rotationSpeed =
    _rotationP2pController.Calculate(currentRotation.value(), targetRotation.value()) * 1_rad_per_s;

  // Clamp translation speed to max velocity
  translationSpeed = std::clamp(
    translationSpeed, -drivebaseConfig::MAX_P2P_VELOCITY, drivebaseConfig::MAX_P2P_VELOCITY);

  // Convert Polar back into Cartesian X and Y
  frc::Translation2d translationSpeedVector =
    frc::Translation2d((translationSpeed.value() * 1_m), translationVector.Angle());
  units::meters_per_second_t xSpeed = translationSpeedVector.X().value() * 1_mps;
  units::meters_per_second_t ySpeed = translationSpeedVector.Y().value() * 1_mps;

  // Limit acceleration
  xSpeed = _xP2pLimiter.Calculate(xSpeed);
  ySpeed = _yP2pLimiter.Calculate(ySpeed);
  rotationSpeed = _rotP2pLimiter.Calculate(rotationSpeed);

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    xSpeed *= -1;
    ySpeed *= -1;
  }

  // Logging
  std::string logPath = "Drivebase/DriveToPose/";
  logger::Log(logPath + "Calc x speed", xSpeed);
  logger::Log(logPath + "Calc y speed", ySpeed);
  logger::Log(logPath + "Calc rot speed", rotationSpeed);
  logger::Log(logPath + "Target pose Y", targetYMeters);
  logger::Log(logPath + "Target pose X", targetXMeters);
  logger::Log(logPath + "Target rotation", targetRotation);
  logger::Log(logPath + "Current pose X", currentXMeters);
  logger::Log(logPath + "Current pose Y", currentYMeters);
  logger::Log(logPath + "Current rotation", currentRotation);
  return frc::ChassisSpeeds{xSpeed, ySpeed, rotationSpeed};
}

bool SubDrivebase::IsAtPose(
  frc::Pose2d pose, units::meter_t posErrorTolerance, units::degree_t rotErrorTolerance) {
  auto currentPose = PoseHandler::GetInstance().GetPose();
  auto rotError = GetGyroAngle(true) - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());

  logger::FieldDisplay::GetInstance().DisplayPose("Drivebase/IsAtPose/Current pose", currentPose);
  logger::FieldDisplay::GetInstance().DisplayPose("Drivebase/IsAtPose/Target pose", pose);

  logger::Log("Drivebase/IsAtPose/rotError", rotError.Degrees());
  logger::Log("Drivebase/IsAtPose/posError", posError);

  bool atPose =
    (units::math::abs(rotError.Degrees()) < rotErrorTolerance) && (posError < posErrorTolerance);
  logger::Log("Drivebase/IsAtPose/IsAtPose", atPose);
  return atPose;
}

frc2::CommandPtr SubDrivebase::DriveToPose(const std::function<frc::Pose2d()>& pose,
  double speedScaling, units::meter_t posErrorTolerance, units::degree_t rotErrorTolerance,
  bool flipForRedAlliance) {
  auto fieldRelativePose = [pose, flipForRedAlliance] {
    return flipForRedAlliance ? icGeometry::GetFieldRelativePose(pose()) : pose();
  };

  return Drive(
    [this, fieldRelativePose, speedScaling] {
      auto pose = fieldRelativePose();
      return CalcDriveToPoseSpeeds(pose) * speedScaling;
    },
    true)
    .Until([this, fieldRelativePose, posErrorTolerance, rotErrorTolerance] {
      return IsAtPose(fieldRelativePose(), posErrorTolerance, rotErrorTolerance);
    });
}

frc::ChassisSpeeds SubDrivebase::CalcJoystickSpeeds(frc2::CommandXboxController& controller) {
  std::string configPath = "Drivebase/Config/";
  auto deadband =
    logger::Tune(configPath + "Joystick Deadband", drivebaseConfig::JOYSTICK_DEADBAND);
  auto maxVelocity = logger::Tune(configPath + "Max Velocity", drivebaseConfig::MAX_VELOCITY);
  auto maxAngularVelocity =
    logger::Tune(configPath + "Max Angular Velocity", drivebaseConfig::MAX_TELEOP_ANGULAR_VELOCITY);
  auto maxJoystickAccel =
    logger::Tune(configPath + "Max Joystick Accel", drivebaseConfig::MAX_JOYSTICK_ACCEL);
  auto maxAngularJoystickAccel = logger::Tune(
    configPath + "Max Joystick Angular Accel", drivebaseConfig::MAX_ANGULAR_JOYSTICK_ACCEL);
  auto translationExponent = logger::Tune(
    configPath + "Joystick Translation Exponent", drivebaseConfig::TRANSLATION_EXPONENT);
  auto rotationExponent =
    logger::Tune(configPath + "Joystick Rotation Exponent", drivebaseConfig::ROTATION_EXPONENT);

  // Recreate slew rate limiters if limits have changed
  if (maxJoystickAccel != _tunedMaxJoystickAccel) {
    _xStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _yStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _tunedMaxJoystickAccel = maxJoystickAccel;
  }
  if (maxAngularJoystickAccel != _tunedMaxAngularJoystickAccel) {
    _rotStickLimiter = frc::SlewRateLimiter<units::scalar>{maxAngularJoystickAccel / 1_s};
    _tunedMaxAngularJoystickAccel = maxAngularJoystickAccel;
  }

  // Apply deadbands
  double rawTranslationY = frc::ApplyDeadband(-controller.GetLeftY(), deadband);
  double rawTranslationX = frc::ApplyDeadband(-controller.GetLeftX(), deadband);
  double rawRotation = frc::ApplyDeadband(-controller.GetRightX(), deadband);

  // Convert cartesian (x, y) translation stick coordinates to polar (R, theta) and scale R-value
  double rawTranslationR = std::min(1.0, sqrt(pow(rawTranslationX, 2) + pow(rawTranslationY, 2)));
  double translationTheta = atan2(rawTranslationY, rawTranslationX);
  double scaledTranslationR = pow(rawTranslationR, translationExponent);

  // Convert polar coordinates (with scaled R-value) back to cartesian; scale rotation as well
  double scaledTranslationY = scaledTranslationR * sin(translationTheta);
  double scaledTranslationX = scaledTranslationR * cos(translationTheta);

  double scaledRotation = pow(rawRotation, rotationExponent);
  // Bring back any negatives that may have been lost by applying the exponent
  if (rawRotation < 0 && scaledRotation > 0) {
    scaledRotation *= 1;
  }

  // Apply joystick rate limits and calculate speed
  auto forwardSpeed = _yStickLimiter.Calculate(scaledTranslationY) * maxVelocity;
  auto sidewaysSpeed = _xStickLimiter.Calculate(scaledTranslationX) * maxVelocity;
  auto rotationSpeed = _rotStickLimiter.Calculate(scaledRotation) * maxAngularVelocity;

  // logger things
  std::string joystickScalingPath = "Drivebase/JoystickScaling/";
  logger::Log(joystickScalingPath + "rawTranslationY", rawTranslationY);
  logger::Log(joystickScalingPath + "rawTranslationX", rawTranslationX);
  logger::Log(joystickScalingPath + "rawTranslationR", rawTranslationR);
  logger::Log(joystickScalingPath + "translationTheta (degrees)",
    translationTheta *
      (180 / std::numbers::pi));  // Multiply by 180/pi to convert radians to degrees
  logger::Log(joystickScalingPath + "scaledTranslationR", scaledTranslationR);
  logger::Log(joystickScalingPath + "scaledTranslationY", scaledTranslationY);
  logger::Log(joystickScalingPath + "scaledTranslationX", scaledTranslationX);
  logger::Log(joystickScalingPath + "rawRotation", rawRotation);
  logger::Log(joystickScalingPath + "scaledRotation", scaledRotation);

  return frc::ChassisSpeeds{forwardSpeed, sidewaysSpeed, rotationSpeed};
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller,
  bool fieldOriented, double speedScaling, double rotationScaling) {
  return Drive(
    [this, speedScaling, rotationScaling, &controller] {
      auto speeds = CalcJoystickSpeeds(controller);
      speeds.vx = speeds.vx * speedScaling;
      speeds.vy = speeds.vy * speedScaling;
      speeds.omega = speeds.omega * rotationScaling;
      return frc::ChassisSpeeds{speeds.vx, speeds.vy, speeds.omega};
    },
    fieldOriented);
}

/* aligns to an a arbitrary angle while allowing joystick driving */
frc2::CommandPtr SubDrivebase::JoystickDriveWithAngle(frc2::CommandXboxController& controller,
  const std::function<units::degree_t()>& target, double speedScaling) {
  return Drive(
    [this, &controller, target, speedScaling] {
      units::angle::degree_t currentAngle = GetGyroAngle(true).Degrees();
      units::turns_per_second_t rotationSpeeds = CalcRotateSpeed(currentAngle - target());
      frc::ChassisSpeeds joystickSpeeds = CalcJoystickSpeeds(controller);

      joystickSpeeds.vx *= speedScaling;
      joystickSpeeds.vy *= speedScaling;
      return frc::ChassisSpeeds(joystickSpeeds.vx, joystickSpeeds.vy, rotationSpeeds);
    },
    true);
}

// Special

frc2::CommandPtr SubDrivebase::CharacteriseWheels() {
  static units::radian_t prevGyroAngle = 0_rad;
  static units::radian_t gyroAccumulator = 0_rad;
  static units::radian_t fRinitialWheelDistance = 0_rad;
  static units::radian_t fLinitialWheelDistance = 0_rad;
  static units::radian_t bRinitialWheelDistance = 0_rad;
  static units::radian_t bLinitialWheelDistance = 0_rad;
  static auto limiter = frc::SlewRateLimiter<units::degrees_per_second>{240_deg_per_s / 10_s};
  static units::meter_t drivebaseRadius = drivebaseConfig::FL_POSITION.Norm();

  return RunOnce([this] {
    prevGyroAngle = GetGyroAngle().Radians();
    gyroAccumulator = 0_rad;
    fRinitialWheelDistance = _frontRight.GetDrivenRotations();
    fLinitialWheelDistance = _frontLeft.GetDrivenRotations();
    bRinitialWheelDistance = _backRight.GetDrivenRotations();
    bLinitialWheelDistance = _backLeft.GetDrivenRotations();
    limiter.Reset(0_deg_per_s);
    logger::Log("Drivebase/WheelCharacterisation/DrivebaseRadius", drivebaseRadius);
  })
    .AndThen(Drive(
      [] {
        auto speed = limiter.Calculate(100_deg_per_s);
        return frc::ChassisSpeeds{0_mps, 0_mps, speed};
      },
      false))
    .AlongWith(frc2::cmd::Wait(1_s).AndThen(frc2::cmd::Run([this] {
      units::radian_t curGyroAngle = GetGyroAngle().Radians();
      gyroAccumulator = gyroAccumulator + frc::AngleModulus((prevGyroAngle - curGyroAngle));
      prevGyroAngle = curGyroAngle;
      logger::Log("Drivebase/WheelCharacterisation/GyroAccum", gyroAccumulator);
      logger::Log("Drivebase/WheelCharacterisation/GyroCur", curGyroAngle);
      logger::Log("Drivebase/WheelCharacterisation/GyroPrev", prevGyroAngle);

      units::radian_t fRfinalWheelDistance = _frontRight.GetDrivenRotations();
      units::radian_t fLfinalWheelDistance = _frontLeft.GetDrivenRotations();
      units::radian_t bRfinalWheelDistance = _backRight.GetDrivenRotations();
      units::radian_t bLfinalWheelDistance = _backLeft.GetDrivenRotations();

      units::radian_t fRdelta = units::math::abs(fRfinalWheelDistance - fRinitialWheelDistance);
      units::radian_t fLdelta = units::math::abs(fLfinalWheelDistance - fLinitialWheelDistance);
      units::radian_t bRdelta = units::math::abs(bRfinalWheelDistance - bRinitialWheelDistance);
      units::radian_t bLdelta = units::math::abs(bLfinalWheelDistance - bLinitialWheelDistance);

      units::radian_t avgWheelDelta = (fRdelta + fLdelta + bRdelta + bLdelta) / 4.0;
      units::meter_t calcedWheelRadius = ((gyroAccumulator * drivebaseRadius) / avgWheelDelta);

      logger::Log("Drivebase/WheelCharacterisation/CalcedWheelRadius", calcedWheelRadius);
      logger::Log("Drivebase/WheelCharacterisation/WheelDistance", avgWheelDelta);

      // logger::Log("Drivebase/WheelCharacterisation/FLinitialWheelDistance",
      // FLinitialWheelDistance);
      // logger::Log("Drivebase/WheelCharacterisation/FRinitialWheelDistance",
      // FRinitialWheelDistance);
      // logger::Log("Drivebase/WheelCharacterisation/BLinitialWheelDistance",
      // BLinitialWheelDistance);
      // logger::Log("Drivebase/WheelCharacterisation/BRinitialWheelDistance",
      // BRinitialWheelDistance);

      // logger::Log("Drivebase/WheelCharacterisation/FLfinalWheelDistance", FLfinalWheelDistance);
      // logger::Log("Drivebase/WheelCharacterisation/FRfinalWheelDistance", FRfinalWheelDistance);
      // logger::Log("Drivebase/WheelCharacterisation/BLfinalWheelDistance", BLfinalWheelDistance);
      // logger::Log("Drivebase/WheelCharacterisation/BRfinalWheelDistance", BRfinalWheelDistance);

      logger::Log("Drivebase/WheelCharacterisation/FLdelta", fLdelta);
      logger::Log("Drivebase/WheelCharacterisation/FRdelta", fRdelta);
      logger::Log("Drivebase/WheelCharacterisation/BLdelta", bLdelta);
      logger::Log("Drivebase/WheelCharacterisation/BRdelta", bRdelta);
    })));
}

void SubDrivebase::SetPose(frc::Pose2d pose) {
  auto states = GetSwerveStates();

  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    ResetGyroHeading(pose.Rotation().Degrees());
  } else {
    ResetGyroHeading(pose.Rotation().Degrees() - 180_deg);
  }

  PoseHandler::GetInstance().SetPose(pose, states);
}