#ifndef PIPE_STREAM_H
#define PIPE_STREAM_H


#ifdef _WIN32
#include <Windows.h>
#else 
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstring>
#endif

#include <filesystem>
#include <string_view>

#include "Log.h"
#include "string_helpers.h"


namespace Navtech::Utility::Shell {

   #ifdef __linux__
    inline void launch(
        const std::filesystem::path& application   [[maybe_unused]],
        std::string_view             options       [[maybe_unused]] = std::string_view { },
        const std::filesystem::path& working_dir   [[maybe_unused]] = std::filesystem::path { "./"} 
    )
    {
        std::string filename { (working_dir/application).string() };

        // execv requires an array of pointers to strings
        // that is, char*[]
        // We have to fabricate this from the - single - option
        // string
        //
        auto option_strings = split(std::string { options }, ' ');
        for (auto& opt : option_strings) trim(opt);

        std::vector<char*> opt_ptrs { };
        for (auto& opt : option_strings) {
            opt_ptrs.emplace_back(&(*opt.data()));
        }
        opt_ptrs.push_back(nullptr);

        // Run the specified program in its own process
        //
        auto pid = ::fork();

        if (pid == 0) {
            // This is the child process; launch the
            // executable
            //
            auto result = ::execv(filename.data(), (char* const*)opt_ptrs.data());

            if (result < 0) {
                stdout_log << Logging_level::error;
                stdout_log << "Could not launch [" << filename << " " << options << "]" << "[" << std::strerror(errno) << "]" << endl;
                stdout_log.pop_level();
            }
        }
        else {
            stdout_log << "Launched [" << filename << " " << options << "] " << "PID [" << pid << "]" << endl;
            
            int status { };
            ::waitpid(pid, &status, 0);
        }
    }

#else
    inline void launch(
        const std::filesystem::path& application, 
        std::string_view             options      = std::string_view { },
        const std::filesystem::path& working_dir  = std::filesystem::path { } 
    )
    {
        std::string filename { application.string() };
        std::string path     { application.parent_path().string() };
        std::string work_dir { working_dir.string() }; 

        // Process-required structures
        //
        STARTUPINFOA        startup_info { };     
        PROCESS_INFORMATION process_info { };
        startup_info.cb = sizeof(startup_info);

        // start the program up
        //
        CreateProcessA( 
            filename.data(),                                    // Application full path
            (!options.empty() ? (LPSTR)options.data() : NULL),  // Command line options
            NULL,                                               // Process handle not inheritable
            NULL,                                               // Thread handle not inheritable
            FALSE,                                              // Set handle inheritance to FALSE
            0,                                                  // No creation flags
            NULL,                                               // Use parent's environment block
            (!work_dir.empty() ? work_dir.data() : NULL),       // Starting directory. NULL => use parent's current dir
            &startup_info,
            &process_info
        );
    }
    
#endif
    
} // namespace Navtech::Utility::Shell

#endif // PIPE_STREAM_H