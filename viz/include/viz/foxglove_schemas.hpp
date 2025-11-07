#pragma once

#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <cstdlib>

namespace viz {
namespace foxglove_schemas {

using json = nlohmann::json;

// Get the runfiles directory from environment or construct it
inline std::string getRunfilesDir() {
  // Try RUNFILES_DIR environment variable first (set by Bazel)
  if (const char* runfiles = std::getenv("RUNFILES_DIR")) {
    return std::string(runfiles);
  }

  // Fallback: construct from executable location
  // Bazel puts runfiles in <binary>.runfiles/
  return "streaming_gs_processor_main.runfiles";
}

/**
 * @brief Schema loader for Foxglove JSON schemas
 *
 * Loads JSON schema files from the foxglove_schemas external dependency.
 * Schemas are loaded lazily on first access and cached.
 */
class SchemaLoader {
 public:
  /**
   * @brief Get a JSON schema by name
   * @param schema_name The schema name (e.g., "foxglove.PointCloud")
   * @return The schema as a JSON string, or std::nullopt if not found
   */
  static std::optional<std::string> getSchema(const std::string& schema_name) {
    static SchemaLoader instance;
    return instance.loadSchema(schema_name);
  }

 private:
  SchemaLoader() = default;

  std::optional<std::string> loadSchema(const std::string& schema_name) {
    // Check cache first
    auto it = schema_cache_.find(schema_name);
    if (it != schema_cache_.end()) {
      return it->second;
    }

    // Map schema name to file path
    std::string filename = getSchemaFilename(schema_name);
    if (filename.empty()) {
      return std::nullopt;
    }

    std::string runfiles = getRunfilesDir();

    // Build schema path using runfiles
    // External dependencies are under: <runfiles>/shadesmar/external/<repo>/
    // Or directly under: <runfiles>/<repo>/
    std::vector<std::string> search_paths = {
        runfiles + "/shadesmar/external/foxglove_schemas/jsonschema/" + filename,
        runfiles + "/foxglove_schemas/jsonschema/" + filename,
        // Fallback for when run from workspace root
        "bazel-bin/gaussian_splatting/" + runfiles + "/shadesmar/external/foxglove_schemas/jsonschema/" + filename,
        "bazel-bin/gaussian_splatting/" + runfiles + "/foxglove_schemas/jsonschema/" + filename,
    };

    for (const auto& path : search_paths) {
      std::ifstream file(path);
      if (file.is_open()) {
        try {
          json schema_json;
          file >> schema_json;

          // Minify and cache
          std::string schema_str = schema_json.dump();
          schema_cache_[schema_name] = schema_str;

          // Debug: print successful path (to stderr to not pollute logs)
          std::cerr << "[SchemaLoader] Loaded " << schema_name << " from: " << path << std::endl;

          return schema_str;
        } catch (const std::exception& e) {
          // Failed to parse JSON, try next path
          std::cerr << "[SchemaLoader] Failed to parse: " << path << " error: " << e.what() << std::endl;
          continue;
        }
      }
    }

    // Schema not found in any path - print tried paths for debugging
    std::cerr << "[SchemaLoader] Failed to load " << schema_name << ", tried:" << std::endl;
    for (const auto& path : search_paths) {
      std::cerr << "  - " << path << std::endl;
    }

    return std::nullopt;
  }

  std::string getSchemaFilename(const std::string& schema_name) {
    // Map common schema names to filenames
    static const std::unordered_map<std::string, std::string> name_to_file = {
        {"foxglove.PointCloud", "PointCloud.json"},
        {"foxglove.CompressedImage", "CompressedImage.json"},
        {"foxglove.Image", "Image.json"},
        {"foxglove.PoseStamped", "PoseStamped.json"},
        {"foxglove.CameraCalibration", "CameraCalibration.json"},
        {"foxglove.FrameTransform", "FrameTransform.json"},
        {"foxglove.Grid", "Grid.json"},
        {"foxglove.SceneUpdate", "SceneUpdate.json"},
    };

    auto it = name_to_file.find(schema_name);
    if (it != name_to_file.end()) {
      return it->second;
    }

    return "";
  }

  std::unordered_map<std::string, std::string> schema_cache_;
};

/**
 * @brief Get schema for a given message type
 * @param schema_name The schema name (e.g., "foxglove.PointCloud")
 * @return The schema as a JSON string, or std::nullopt if not found
 */
inline std::optional<std::string> getSchema(const std::string& schema_name) {
  return SchemaLoader::getSchema(schema_name);
}

}  // namespace foxglove_schemas
}  // namespace viz
