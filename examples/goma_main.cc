// Copyright (c) 2023, Assia Benbihi
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of the <organization> nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: Assia Benbihi (abenbihi-at-georgiatech-hyphen-metz-dot-fr)
//                       (assia-dot-benbihi-at-cvut-dot-cz)

#include <stdio.h>
#include <stdlib.h>
#include <functional>
#include <iostream>
#include <vector>

#include "examples/image.h"
#include "examples/two_views.h"

typedef std::function<int(int, char**)> command_func_t;

int ShowHelp(
    const std::vector<std::pair<std::string, command_func_t>>& commands) {
  std::cout << "GoMa Geometry\n" << std::endl << std::endl;

  std::cout << "Usage:" << std::endl;
  std::cout << "  goma [command] [options]" << std::endl << std::endl;

  std::cout << "Example usage:" << std::endl;
  std::cout << "  goma help [ -h, --help ]" << std::endl;
  std::cout << "  goma line_segment_detector -h [ --help ]" << std::endl;
  std::cout << "  ..." << std::endl << std::endl;

  std::cout << "Available commands:" << std::endl;
  std::cout << "  help" << std::endl;
  for (const auto& command : commands) {
    std::cout << "  " << command.first << std::endl;
  }
  std::cout << std::endl;

  return EXIT_SUCCESS;
}

int main(int argc, char* argv[]){
  std::vector<std::pair<std::string, command_func_t>> commands;

  commands.emplace_back(std::pair<std::string, command_func_t>(
        "line_segment_detector", &goma::RunLineSegmentDetector));

  commands.emplace_back(std::pair<std::string, command_func_t>(
      "vanishing_point_detector", &goma::RunVanishingPointDetector));

  commands.emplace_back(std::pair<std::string, command_func_t>(
      "planewise_homography_estimator", &goma::RunPlanewiseHomographyEstimation));

  if (argc == 1) {
    return ShowHelp(commands);
  }

  const std::string command = argv[1];
  command_func_t matched_command_func = nullptr;
  for (const auto& command_func : commands) {
    if (command == command_func.first) {
      matched_command_func = command_func.second;
      break;
    }
  }
  if (matched_command_func == nullptr) {
    std::cerr << 
        "ERROR: Command " << command << "not recognized. To list the "
        "available commands, run `colmap help`."
      << std::endl;
    return EXIT_FAILURE;
  } else {
    int command_argc = argc - 1;
    char** command_argv = &argv[1];
    command_argv[0] = argv[0];
    return matched_command_func(command_argc, command_argv);
  }

  return 0;
}
