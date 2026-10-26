#include "rover/vehicle_config.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

namespace rover {

namespace {

// Same job as Python's str.strip(): remove leading and trailing whitespace.
std::string trimWhitespace(const std::string& text) {
  const char* const whitespaceCharacters = " \t\r\n";
  const std::size_t firstKeptIndex = text.find_first_not_of(whitespaceCharacters);
  if (firstKeptIndex == std::string::npos) {
    return "";  // the string was empty or all whitespace
  }
  const std::size_t lastKeptIndex = text.find_last_not_of(whitespaceCharacters);
  return text.substr(firstKeptIndex, lastKeptIndex - firstKeptIndex + 1);
}

// Like Python's float(), but rejects trailing junk ("1.5abc") and nan/inf,
// and reports which file/line was bad instead of a bare parse error.
double parseFiniteNumber(const std::string& valueText, const std::string& key,
                         const std::string& filePath, int lineNumber) {
  const std::string whereText = filePath + ":" + std::to_string(lineNumber) + ": key '" + key + "'";
  std::size_t charactersConsumed = 0;
  double parsedValue = 0.0;
  try {
    parsedValue = std::stod(valueText, &charactersConsumed);
  } catch (const std::exception&) {
    // std::stod throws std::invalid_argument / std::out_of_range on bad input.
    throw std::runtime_error(whereText + " has non-numeric value '" + valueText + "'");
  }
  if (charactersConsumed != valueText.size() || !std::isfinite(parsedValue)) {
    throw std::runtime_error(whereText + " has non-numeric value '" + valueText + "'");
  }
  return parsedValue;
}

}  // namespace

VehicleConfig VehicleConfig::loadFromFile(const std::string& filePath) {
  std::ifstream inputFile(filePath);
  if (!inputFile.is_open()) {
    throw std::runtime_error("Cannot open vehicle config file: " + filePath);
  }

  VehicleConfig loadedConfig;
  std::string rawLine;
  int lineNumber = 0;

  while (std::getline(inputFile, rawLine)) {
    ++lineNumber;

    // Drop everything from the first '#' onward, then trim. What is left is
    // either empty (blank/comment-only line) or a `key = value` pair.
    const std::size_t commentStartIndex = rawLine.find('#');
    if (commentStartIndex != std::string::npos) {
      rawLine.erase(commentStartIndex);
    }
    const std::string lineWithoutComment = trimWhitespace(rawLine);
    if (lineWithoutComment.empty()) {
      continue;
    }

    const std::size_t equalsSignIndex = lineWithoutComment.find('=');
    if (equalsSignIndex == std::string::npos) {
      throw std::runtime_error(filePath + ":" + std::to_string(lineNumber) +
                               ": expected 'key = value', got '" + lineWithoutComment + "'");
    }
    const std::string key = trimWhitespace(lineWithoutComment.substr(0, equalsSignIndex));
    const std::string valueText = trimWhitespace(lineWithoutComment.substr(equalsSignIndex + 1));

    if (key == "name") {
      loadedConfig.name = valueText;
    } else if (key == "wheelbase_meters") {
      loadedConfig.wheelbaseMeters = parseFiniteNumber(valueText, key, filePath, lineNumber);
    } else if (key == "rear_track_width_meters") {
      loadedConfig.rearTrackWidthMeters = parseFiniteNumber(valueText, key, filePath, lineNumber);
    } else if (key == "tire_diameter_meters") {
      loadedConfig.tireDiameterMeters = parseFiniteNumber(valueText, key, filePath, lineNumber);
    } else if (key == "meters_per_encoder_tick") {
      loadedConfig.metersPerEncoderTick = parseFiniteNumber(valueText, key, filePath, lineNumber);
    } else if (key == "max_steering_angle_radians") {
      loadedConfig.maxSteeringAngleRadians = parseFiniteNumber(valueText, key, filePath, lineNumber);
    } else if (key == "rear_axle_locked") {
      if (valueText != "true" && valueText != "false") {
        throw std::runtime_error(filePath + ":" + std::to_string(lineNumber) + ": key '" + key +
                                 "' must be true or false, got '" + valueText + "'");
      }
      loadedConfig.rearAxleLocked = valueText == "true";
    }
    // Any other key is deliberately ignored (forward compatibility).
  }

  return loadedConfig;
}

}  // namespace rover
