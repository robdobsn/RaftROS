/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS link selection - shared by every backend so application code (an
// API that switches the link, say) builds the same whatever the transport.
//
// The Zenoh build reaches its router over TCP (WiFi) or over the z-serial
// link on the chip's USB-Serial-JTAG port.  The other builds have only their
// IP transport and refuse a change, saying why.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

enum class RaftROSLinkType : uint8_t
{
    TCP,        ///< IP: TCP to the router (Zenoh) - the default
    USB_JTAG,   ///< z-serial frames over the chip's USB-Serial-JTAG port
};

/// @brief A requested change to the link.  Only the fields marked "has" change.
struct RaftROSLinkConfig
{
    bool hasLink = false;
    RaftROSLinkType link = RaftROSLinkType::TCP;
    bool hasLinkLogs = false;
    bool linkLogs = false;      ///< copy log lines onto a serial link (bridge tool only)
    bool persist = false;       ///< also write the change to the settings overlay (NVS)
};

/// @brief Name used in configuration and the API ("tcp", "usbjtag")
inline const char* RaftROSLink_name(RaftROSLinkType link)
{
    return link == RaftROSLinkType::USB_JTAG ? "usbjtag" : "tcp";
}

/// @brief Parse a link name.  @return false if the name is not one
inline bool RaftROSLink_fromName(const char* name, RaftROSLinkType& link)
{
    if (!name)
        return false;
    if (strcasecmp(name, "tcp") == 0)
    {
        link = RaftROSLinkType::TCP;
        return true;
    }
    if (strcasecmp(name, "usbjtag") == 0)
    {
        link = RaftROSLinkType::USB_JTAG;
        return true;
    }
    return false;
}
