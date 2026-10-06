// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_ASSETS_HPP_
#define TEST_ASSETS_HPP_

#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <string>

#include "urdf_parser/urdf_parser.hpp"

// Parses a URDF from the test assets directory.
// Unlike urdf::parseURDFFile, an asset that cannot be opened is a test failure
// rather than a null model, so tests expecting a parse error cannot pass on a
// wrong path.
inline urdf::ModelInterfaceSharedPtr parseAsset(const std::string & name)
{
  const std::string path = std::string(TEST_ASSETS_DIR) + "/" + name;
  std::ifstream stream(path.c_str());
  if (!stream) {
    ADD_FAILURE() << "Test asset " << path << " could not be opened";
    return nullptr;
  }
  const std::string xml_str((std::istreambuf_iterator<char>(stream)),
    std::istreambuf_iterator<char>());
  return urdf::parseURDF(xml_str);
}

#endif  // TEST_ASSETS_HPP_
