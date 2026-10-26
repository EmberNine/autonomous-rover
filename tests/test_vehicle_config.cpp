#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "rover/bicycle_model.hpp"
#include "rover/vehicle_config.hpp"

using rover::VehicleConfig;

namespace {

// Writes text to a uniquely named file in the system temp directory and
// deletes it when the object goes out of scope (like a Python `with` block /
// tempfile.NamedTemporaryFile). This is the C++ "RAII" idiom: cleanup lives
// in the destructor, so it runs even if a test assertion bails out early.
class TemporaryConfigFile {
 public:
  explicit TemporaryConfigFile(const std::string& fileContents) {
    static std::atomic<int> fileCounter{0};
    filePath_ = std::filesystem::temp_directory_path() /
                ("rover_test_config_" + std::to_string(fileCounter++) + ".cfg");
    std::ofstream outputFile(filePath_);
    outputFile << fileContents;
  }
  ~TemporaryConfigFile() {
    std::error_code ignoredError;
    std::filesystem::remove(filePath_, ignoredError);
  }
  TemporaryConfigFile(const TemporaryConfigFile&) = delete;
  TemporaryConfigFile& operator=(const TemporaryConfigFile&) = delete;

  std::string path() const { return filePath_.string(); }

 private:
  std::filesystem::path filePath_;
};

}  // namespace

TEST(VehicleConfigParsing, ReadsAllKnownKeys) {
  TemporaryConfigFile configFile(
      "name = Test Rig\n"
      "wheelbase_meters = 0.25\n"
      "rear_track_width_meters = 0.09\n"
      "tire_diameter_meters = 0.06\n"
      "meters_per_encoder_tick = 0.002\n"
      "max_steering_angle_radians = 0.4\n");
  const VehicleConfig loadedConfig = VehicleConfig::loadFromFile(configFile.path());
  EXPECT_EQ(loadedConfig.name, "Test Rig");
  EXPECT_DOUBLE_EQ(loadedConfig.wheelbaseMeters, 0.25);
  EXPECT_DOUBLE_EQ(loadedConfig.rearTrackWidthMeters, 0.09);
  EXPECT_DOUBLE_EQ(loadedConfig.tireDiameterMeters, 0.06);
  EXPECT_DOUBLE_EQ(loadedConfig.metersPerEncoderTick, 0.002);
  EXPECT_DOUBLE_EQ(loadedConfig.maxSteeringAngleRadians, 0.4);
}

TEST(VehicleConfigParsing, IgnoresCommentsAndBlankLines) {
  TemporaryConfigFile configFile(
      "# a full-line comment\n"
      "\n"
      "   \t  \n"
      "wheelbase_meters = 0.25   # trailing comment\n"
      "   # indented comment\n"
      "# wheelbase_meters = 99.0   (commented out, must not apply)\n");
  const VehicleConfig loadedConfig = VehicleConfig::loadFromFile(configFile.path());
  EXPECT_DOUBLE_EQ(loadedConfig.wheelbaseMeters, 0.25);
}

TEST(VehicleConfigParsing, ToleratesWhitespaceAroundKeyAndValue) {
  TemporaryConfigFile configFile("   wheelbase_meters\t=   0.25   \r\n");
  const VehicleConfig loadedConfig = VehicleConfig::loadFromFile(configFile.path());
  EXPECT_DOUBLE_EQ(loadedConfig.wheelbaseMeters, 0.25);
}

TEST(VehicleConfigParsing, IgnoresUnknownKeys) {
  // track_width_meters is deliberately included here: it used to be a
  // recognized key (a single, undifferentiated track width) but is now
  // front_track_width_meters/rear_track_width_meters instead, both
  // meaningless to this ROS-free core (see VehicleConfig's header comment).
  // This guards against ever silently reviving it as a recognized key.
  TemporaryConfigFile configFile(
      "wheelbase_meters = 0.25\n"
      "track_width_meters = 0.15\n"
      "flux_capacitor_gigawatts = 1.21\n"
      "paint_color = olive\n");
  VehicleConfig loadedConfig;
  EXPECT_NO_THROW(loadedConfig = VehicleConfig::loadFromFile(configFile.path()));
  EXPECT_DOUBLE_EQ(loadedConfig.wheelbaseMeters, 0.25);
}

TEST(VehicleConfigParsing, MissingKeysKeepDefaults) {
  TemporaryConfigFile configFile("wheelbase_meters = 0.25\n");
  const VehicleConfig loadedConfig = VehicleConfig::loadFromFile(configFile.path());
  EXPECT_EQ(loadedConfig.name, "unnamed_vehicle");
  EXPECT_DOUBLE_EQ(loadedConfig.tireDiameterMeters, 0.0);
}

TEST(VehicleConfigParsing, EmptyFileYieldsDefaults) {
  TemporaryConfigFile configFile("");
  EXPECT_NO_THROW(VehicleConfig::loadFromFile(configFile.path()));
}

TEST(VehicleConfigParsing, LaterDuplicateKeyWins) {
  TemporaryConfigFile configFile("wheelbase_meters = 0.1\nwheelbase_meters = 0.2\n");
  EXPECT_DOUBLE_EQ(VehicleConfig::loadFromFile(configFile.path()).wheelbaseMeters, 0.2);
}

TEST(VehicleConfigParsing, ReadsRearAxleLockedAndDefaultsToOpen) {
  EXPECT_FALSE(VehicleConfig::loadFromFile(TemporaryConfigFile("").path()).rearAxleLocked);
  EXPECT_TRUE(VehicleConfig::loadFromFile(TemporaryConfigFile("rear_axle_locked = true\n").path()).rearAxleLocked);
  EXPECT_FALSE(VehicleConfig::loadFromFile(TemporaryConfigFile("rear_axle_locked = false # open\n").path())
                   .rearAxleLocked);
}

TEST(VehicleConfigErrors, RearAxleLockedMustBeTrueOrFalse) {
  TemporaryConfigFile configFile("rear_axle_locked = yes\n");
  EXPECT_THROW(VehicleConfig::loadFromFile(configFile.path()), std::runtime_error);
}

TEST(VehicleConfigErrors, MissingFileThrowsRuntimeError) {
  const std::string nonexistentPath =
      (std::filesystem::temp_directory_path() / "rover_definitely_not_here.cfg").string();
  EXPECT_THROW(VehicleConfig::loadFromFile(nonexistentPath), std::runtime_error);
}

TEST(VehicleConfigErrors, NonNumericValueThrowsRuntimeError) {
  TemporaryConfigFile configFile("wheelbase_meters = five point two\n");
  EXPECT_THROW(VehicleConfig::loadFromFile(configFile.path()), std::runtime_error);
}

TEST(VehicleConfigErrors, TrailingGarbageAfterNumberThrowsRuntimeError) {
  TemporaryConfigFile configFile("wheelbase_meters = 0.25abc\n");
  EXPECT_THROW(VehicleConfig::loadFromFile(configFile.path()), std::runtime_error);
}

TEST(VehicleConfigErrors, LineWithoutEqualsSignThrowsRuntimeError) {
  TemporaryConfigFile configFile("wheelbase_meters 0.25\n");
  EXPECT_THROW(VehicleConfig::loadFromFile(configFile.path()), std::runtime_error);
}

TEST(VehicleConfigErrors, UnknownKeyWithNonNumericValueStillIgnored) {
  TemporaryConfigFile configFile("some_future_key = not a number\n");
  EXPECT_NO_THROW(VehicleConfig::loadFromFile(configFile.path()));
}

// Guards against the shipped config file rotting out of sync with the parser.
TEST(ShippedVehicleConfig, LoadsAndProducesAValidBicycleModel) {
  const std::string shippedConfigPath = std::string(ROVER_SOURCE_DIR) + "/config/vehicle.cfg";
  const VehicleConfig loadedConfig = VehicleConfig::loadFromFile(shippedConfigPath);
  EXPECT_NEAR(loadedConfig.wheelbaseMeters, 0.132, 0.01);  // ~5.2 in
  EXPECT_DOUBLE_EQ(loadedConfig.metersPerEncoderTick, 3.9336e-5);  // MEASURED 2026-09-29, avg 25422 counts/m
  EXPECT_DOUBLE_EQ(loadedConfig.rearTrackWidthMeters, 0.081);
  EXPECT_TRUE(loadedConfig.rearAxleLocked);  // CONFIRMED 2026-10-25
  EXPECT_NO_THROW(rover::BicycleModel model(loadedConfig));
}
