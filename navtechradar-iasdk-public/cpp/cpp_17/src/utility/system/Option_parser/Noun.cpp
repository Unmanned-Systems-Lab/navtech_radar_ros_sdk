// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2024 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "Noun.h"

namespace Navtech::Utility {

    // ------------------------------------------------------------------------
    // Constructors/Destructors
    //
    Noun::Noun(std::string_view name, std::initializer_list<Option> options) :
    noun_name { name },
    options { options }
    {
    }

    
    // ------------------------------------------------------------------------
    // Public methods
    //
    Noun& Noun::add_option(const Option option)
    {
        options.emplace_back(option);
        return *this;
    }


    Noun& Noun::add_option(Option&& option)
    {
        options.push_back(std::move(option));
        return *this;
    }


    bool Noun::operator==(std::string_view noun) const
    {
        return noun == noun_name;
    }


    const Option& Noun::operator[](std::string_view option) const
    {
        auto itr = std::find(options.begin(), options.end(), option);

        if (itr == options.end()) {
            throw std::invalid_argument("Unknown option: " + std::string { option });
        }

        return *itr;
    }


    const Option& Noun::operator[](const char* option) const
    {
        return operator[](std::string_view { option });
    }


    std::string Noun::usage() const
    {
        std::stringstream oss { };

        oss << "[" << (noun_name.empty() ? "global" : noun_name) << "]\n";
        for (auto& option : options) {
            oss << std::left << "\t" << option.usage() << "\n";
        }

        return oss.str();
    }


    std::string Noun::help() const
    {
        std::stringstream oss { };
        oss << std::left << "[" << (noun_name.empty() ? "global" : noun_name) << "]\n";
        for (auto& option : options) {
            oss << std::left << option.help() << '\n';
        }

        return oss.str();
    }


    void Noun::parse(const std::vector<std::string>& tokens)
    {
        if (tokens.size() == 0) return;
        auto current_option = options.begin();

        // Early get out, in case there are no options given
        //

        if (std::find(options.begin(), options.end(), tokens[0]) == options.end()) {
            std::stringstream oss { };
            oss << "[" << tokens[0] << "] is not a recognised ";
            
            if (noun_name.empty()) {
                oss << "global option";
            } 
            else {
                oss << "option for command [" << noun_name << "]";
            }

            throw std::invalid_argument(oss.str());
        }

        try {
            for (auto& token : tokens) {
                auto option_itr = std::find(options.begin(), options.end(), token);

                if (option_itr != options.end()) {
                    current_option = option_itr;
                    if (!current_option->has_args()) {
                        current_option->value("");
                    }
                    continue;
                }

                current_option->value(token);
            }
        }
        catch (const std::exception& ex) {
            throw std::invalid_argument(
                "[" + noun_name + "]: " + ex.what()
            );
        }
    }


    void Noun::check()
    {
        for (auto& option : options) {
            try
            {
                option.check();
            }
            catch(const std::exception& ex)
            {
                std::ostringstream oss { };
                oss << (noun_name != "" ? "Command [" + noun_name + "]: " : "");
                oss << ex.what();
                throw std::invalid_argument(oss.str());
            }
            
        }
    }
} // namespace Navtech::Utilitygit 