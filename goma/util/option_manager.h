// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
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
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
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
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)
//
// Editor: Assia Benbihi (abenbihi-at-georgiatech-hyphen-metz-dot-fr)
//                       (assia-dot-benbihi-at-cvut-dot-cz)
// Edits: Parameters and options registrations for the options specific to
// goma. 

#ifndef GOMA_UTIL_OPTION_MANAGER_H
#define GOMA_UTIL_OPTION_MANAGER_H

#include <boost/program_options.hpp>

#include "util/logging.h"

#include "goma/feature/types.h"

namespace goma {

struct FeatureMatchingOptions;
struct ReaderOptions;
struct VanishingPointOptions;

class OptionManager {
  public:
    OptionManager();

    void AddFeatureMatchingOptions();
    void AddLogOptions();
    void AddReaderOptions();
    void AddVanishingPointOptions();

    void Reset();
    void ResetOptions();
  
    bool Check();
  
    void Parse(const int argc, char** argv);
    bool Read(const std::string& path);
    void Write(const std::string& path) const;

    std::shared_ptr<FeatureMatchingOptions> feature_matching;
    std::shared_ptr<ReaderOptions> reader;
    std::shared_ptr<VanishingPointOptions> vanishing_point;

  private:
    template <typename T>
      void AddAndRegisterRequiredOption(const std::string& name, T* option,
          const std::string& help_text = "");
    template <typename T>
      void AddAndRegisterDefaultOption(const std::string& name, T* option,
          const std::string& help_text = "");
  
    template <typename T>
      void RegisterOption(const std::string& name, const T* option);

    std::shared_ptr<boost::program_options::options_description> desc_;

    std::vector<std::pair<std::string, const bool*>> options_bool_;
    std::vector<std::pair<std::string, const int*>> options_int_;
    std::vector<std::pair<std::string, const double*>> options_double_;
    std::vector<std::pair<std::string, const std::string*>> options_string_;

    bool added_feature_matching_options_;
    bool added_reader_options_;
    bool added_vanishing_point_options_;
    bool added_log_options_;

};

template <typename T>
void OptionManager::AddAndRegisterRequiredOption(const std::string& name,
                                                 T* option,
                                                 const std::string& help_text) {
  desc_->add_options()(name.c_str(),
                       boost::program_options::value<T>(option)->required(),
                       help_text.c_str());
  RegisterOption(name, option);
}

template <typename T>
void OptionManager::AddAndRegisterDefaultOption(const std::string& name,
                                                T* option,
                                                const std::string& help_text) {
  desc_->add_options()(
      name.c_str(),
      boost::program_options::value<T>(option)->default_value(*option),
      help_text.c_str());
  RegisterOption(name, option);
}

template <typename T>
void OptionManager::RegisterOption(const std::string& name, const T* option) {
  //std::cout << name << std::endl;
  if (std::is_same<T, bool>::value) {
    //std::cout << "is_bool" << std::endl;
    options_bool_.emplace_back(name, reinterpret_cast<const bool*>(option));
  } else if (std::is_same<T, int>::value) {
    //std::cout << "is_int" << std::endl;
    options_int_.emplace_back(name, reinterpret_cast<const int*>(option));
  } else if (std::is_same<T, double>::value) {
    //std::cout << "is_double" << std::endl;
    options_double_.emplace_back(name, reinterpret_cast<const double*>(option));
  } else if (std::is_same<T, std::string>::value) {
    //std::cout << "is_string: " << *option << std::endl;
    options_string_.emplace_back(name,
                                 reinterpret_cast<const std::string*>(option));
  } else {
    //std::cout << *option << std::endl;
    LOG(FATAL) << "Unsupported option type";
  }
}

} // namespace goma

#endif // GOMA_OPTION_MANAGER_H
