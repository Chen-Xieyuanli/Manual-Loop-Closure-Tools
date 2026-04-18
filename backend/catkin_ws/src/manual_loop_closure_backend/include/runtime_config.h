#ifndef MS_MAPPING_RUNTIME_CONFIG_H_
#define MS_MAPPING_RUNTIME_CONFIG_H_

#include <string>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <ros/ros.h>

inline std::string &LogDirectoryStorage()
{
    static std::string directory;
    return directory;
}

inline void SetLogDirectory(const std::string &directory)
{
    auto &storage = LogDirectoryStorage();
    storage = directory;
}

inline const std::string &GetLogDirectory()
{
    return LogDirectoryStorage();
}

inline std::string GenerateLogTimestampString()
{
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t tt = clock::to_time_t(now);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &tt);
#else
    localtime_r(&tt, &tm_buf);
#endif
    char buffer[32];
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &tm_buf) == 0)
    {
        return "0000-00-00_00-00-00";
    }
    return std::string(buffer);
}

inline std::string EnsureLogDirectoryStamp()
{
    std::string stamp;
    ros::param::get("/ms_mapping/log_directory_stamp", stamp);

    std::string owner;
    ros::param::get("/ms_mapping/log_directory_stamp_owner", owner);

    std::string launch_uuid;
    ros::param::get("/roslaunch/uuid", launch_uuid);

    bool need_new_stamp = stamp.empty();
    if (!launch_uuid.empty())
    {
        need_new_stamp = need_new_stamp || (owner != launch_uuid);
    }

    if (need_new_stamp)
    {
        stamp = GenerateLogTimestampString();
        ros::param::set("/ms_mapping/log_directory_stamp", stamp);
        if (!launch_uuid.empty())
        {
            ros::param::set("/ms_mapping/log_directory_stamp_owner", launch_uuid);
        }
    }

    return stamp;
}

inline std::string ComposeLogDirectory(const std::string &base_directory,
                                       const std::string &sequence)
{
    std::string path = base_directory;
    if (!path.empty() && path.back() != '/')
    {
        path.push_back('/');
    }

    path += sequence;
    if (!path.empty() && path.back() != '/')
    {
        path.push_back('/');
    }

    path += EnsureLogDirectoryStamp();
    if (!path.empty() && path.back() != '/')
    {
        path.push_back('/');
    }
    return path;
}

inline bool MakeDirs(const std::string &dir)
{
    if (dir.empty()) return false;
    std::string path;
    path.reserve(dir.size());
    for (size_t i = 0; i < dir.size(); ++i)
    {
        path.push_back(dir[i]);
        if (dir[i] == '/' || i == dir.size() - 1)
        {
            if (path.empty()) continue;
            // skip root '/'
            if (path.size() == 1 && path[0] == '/') continue;
            struct stat st{};
            if (stat(path.c_str(), &st) != 0)
            {
                // try create
                if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

inline std::string DebugFilePath(const std::string &name)
{
    auto &base = LogDirectoryStorage();
    if (base.empty())
    {
        return std::string(ROOT_DIR) + name;
    }

    std::string path = base;
    if (!path.empty() && path.back() != '/' && path.back() != '\\')
    {
        path.push_back('/');
    }

    MakeDirs(path);

    return path + name;
}

#endif
