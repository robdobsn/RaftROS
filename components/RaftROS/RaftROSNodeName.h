/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS node naming - shared by every backend.
//
// With no "nodeName" configured the node is named after the device: Raft's
// friendly name, which is the name set with /api/friendlyname or, until one is
// set, the SysType's DefaultName (SystemName if none) with the last six hex
// digits of the MAC - so two boards built from the same image are two
// distinct ROS nodes.  The name is made legal for ROS (letters, digits and
// underscores, not starting with a digit) and lower-cased by convention.
//
// Service names given to addService may be relative to the node: "~/devices"
// becomes "<namespace>/<node>/devices", as in rclcpp.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftArduino.h"
#include <cctype>

/// @brief Make a ROS-legal node name: [a-z0-9_], not starting with a digit,
/// runs of illegal characters collapsed to one underscore
inline String RaftROSNodeName_sanitise(const String& name)
{
    String out;
    bool lastUnderscore = false;
    for (unsigned index = 0; index < name.length(); ++index)
    {
        const char c = name[index];
        if (isalnum((unsigned char)c))
        {
            out += (char)tolower((unsigned char)c);
            lastUnderscore = false;
        }
        else if (!lastUnderscore && out.length() > 0)
        {
            out += '_';
            lastUnderscore = true;
        }
    }
    while (out.endsWith("_"))
        out = out.substring(0, out.length() - 1);
    if (out.length() == 0)
        out = "raftros";
    if (isdigit((unsigned char)out[0]))
        out = "n" + out;
    return out;
}

/// @brief "<ns>/<node>" with a single slash between, for the root namespace too
inline String RaftROSNodeName_path(const String& nodeNamespace, const String& nodeName)
{
    String path = nodeNamespace.length() ? nodeNamespace : String("/");
    if (!path.startsWith("/"))
        path = "/" + path;
    if (!path.endsWith("/"))
        path += "/";
    return path + nodeName;
}

/// @brief Expand a service name: "~/x" is relative to the node; anything else
/// is absolute (a missing leading slash is added)
inline String RaftROSNodeName_expandService(const char* name, const String& nodeNamespace, const String& nodeName)
{
    if (!name)
        return "";
    if (name[0] == '~')
    {
        const char* rest = name + 1;
        if (*rest == '/')
            ++rest;
        return RaftROSNodeName_path(nodeNamespace, nodeName) + "/" + rest;
    }
    return name[0] == '/' ? String(name) : String("/") + name;
}
