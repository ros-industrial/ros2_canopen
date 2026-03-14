//    Copyright 2026 Tim Clephas, Nobleo Autonomous Solutions B.V.
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace ros2_canopen::detail
{

/**
 * @brief Expand environment variables in a string.
 *
 * Replaces ${VAR} or $VAR patterns with the corresponding environment variable value.
 *
 * @param input String potentially containing environment variable references
 * @return String with environment variables expanded
 */
inline std::string expand_environment_variables(const std::string & input)
{
  std::string result = input;

  // Match ${VAR} pattern
  std::regex env_regex_braces(R"(\$\{([^}]+)\})");
  std::smatch match;
  while (std::regex_search(result, match, env_regex_braces))
  {
    const char * env_value = std::getenv(match[1].str().c_str());
    std::string replacement = env_value ? env_value : "";
    result = match.prefix().str() + replacement + match.suffix().str();
  }

  // Match $VAR pattern
  std::regex env_regex_plain(R"(\$([A-Za-z_][A-Za-z0-9_]*))");
  while (std::regex_search(result, match, env_regex_plain))
  {
    const char * env_value = std::getenv(match[1].str().c_str());
    std::string replacement = env_value ? env_value : "";
    result = match.prefix().str() + replacement + match.suffix().str();
  }

  return result;
}

/**
 * @brief Resolve a potentially relative file path to an absolute path.
 *
 * If the path is relative, resolves it relative to base_dir.
 * Also expands environment variables.
 *
 * @param path Path to resolve (may be relative or absolute)
 * @param base_dir Base directory for resolving relative paths
 * @return Absolute path string
 */
inline std::string resolve_file_path(
  const std::string & path, const std::filesystem::path & base_dir)
{
  // First expand any environment variables
  std::string expanded = expand_environment_variables(path);

  std::filesystem::path fs_path(expanded);

  // If relative, make it relative to base_dir
  if (fs_path.is_relative())
  {
    fs_path = base_dir / fs_path;
  }

  // Normalize the path (resolve . and ..)
  try
  {
    return std::filesystem::canonical(fs_path).string();
  }
  catch (const std::filesystem::filesystem_error &)
  {
    // If canonical fails (e.g., path doesn't exist), use weakly_canonical
    return std::filesystem::weakly_canonical(fs_path).string();
  }
}

/**
 * @brief Preprocess DCF and write to a temporary file.
 *
 * Creates a temp file with preprocessed content (resolved UploadFile/DownloadFile paths).
 * If no relative paths are found, returns the original path unchanged.
 *
 * @param dcf_path Path to the original DCF file
 * @return Path to the preprocessed temporary file, or original path if no changes needed.
 */
inline std::string preprocess_dcf_to_temp(const std::string & dcf_path)
{
  std::filesystem::path dcf_file(dcf_path);
  std::filesystem::path dcf_dir = dcf_file.parent_path();
  if (dcf_dir.empty())
  {
    dcf_dir = std::filesystem::current_path();
  }

  std::ifstream infile(dcf_path);
  if (!infile.is_open())
  {
    throw std::runtime_error("Failed to open DCF file: " + dcf_path);
  }

  std::stringstream result;
  std::string line;
  bool needs_preprocessing = false;

  // Regex to match UploadFile= or DownloadFile= entries
  std::regex file_entry_regex(R"(^(UploadFile|DownloadFile)=(.*)$)", std::regex::icase);

  while (std::getline(infile, line))
  {
    std::smatch match;
    if (std::regex_match(line, match, file_entry_regex))
    {
      std::string key = match[1].str();
      std::string value = match[2].str();
      std::string resolved = resolve_file_path(value, dcf_dir);

      if (resolved != value)
      {
        needs_preprocessing = true;
      }
      result << key << "=" << resolved << "\n";
    }
    else
    {
      result << line << "\n";
    }
  }

  if (!needs_preprocessing)
  {
    return dcf_path;  // No changes needed
  }

  // Create temp file
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::filesystem::path temp_file =
    temp_dir / ("canopen_" + std::to_string(std::hash<std::string>{}(dcf_path)) + "_" +
                dcf_file.filename().string());

  std::ofstream outfile(temp_file);
  if (!outfile.is_open())
  {
    throw std::runtime_error("Failed to create temp DCF file: " + temp_file.string());
  }

  outfile << result.str();
  return temp_file.string();
}

}  // namespace ros2_canopen::detail
