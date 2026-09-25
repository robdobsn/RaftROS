/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ZenohInterestMatch - does a router's interest cover one of our keys?
//
// A router that wants to know what this node holds sends an interest carrying
// a key expression; every declaration we hold whose key the expression covers
// must be replied with, followed by a final.  Answering an expression we do not
// fully understand would under-report our declarations and leave the router
// with a wrong view of the graph, so anything beyond an empty ("everything")
// expression, an exact key, or a `<prefix>/**` subtree is reported as
// Unsupported for the caller to refuse rather than guess at.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <string_view>

namespace RaftRuntime::Zenoh
{

enum class ZenohInterestMatchResult : uint8_t
{
    NoMatch,        ///< The expression is understood and does not cover this key
    Match,          ///< The expression covers this key - reply with its declaration
    Unsupported,    ///< The expression uses wildcards we do not implement
};

/// @brief Decide whether an interest's key expression covers one of our keys
/// @param pattern key expression from the interest; empty means "everything"
/// @param key one of our declared keys (a liveliness token or topic key)
inline ZenohInterestMatchResult ZenohInterestMatch(std::string_view pattern, std::string_view key)
{
    if (pattern.empty())
        return ZenohInterestMatchResult::Match;
    if (pattern.size() >= 3 && pattern.substr(pattern.size() - 3) == "/**")
    {
        const auto prefix = pattern.substr(0, pattern.size() - 3);
        if (prefix.find_first_of("*$?") != std::string_view::npos)
            return ZenohInterestMatchResult::Unsupported;
        // `<prefix>/**` covers the prefix itself and everything below it, but
        // not a sibling that merely starts with the same characters
        if (key == prefix)
            return ZenohInterestMatchResult::Match;
        return key.size() > prefix.size() && key.substr(0, prefix.size()) == prefix &&
               key[prefix.size()] == '/' ?
               ZenohInterestMatchResult::Match : ZenohInterestMatchResult::NoMatch;
    }
    if (pattern.find_first_of("*$?") != std::string_view::npos)
        return ZenohInterestMatchResult::Unsupported;
    return pattern == key ? ZenohInterestMatchResult::Match : ZenohInterestMatchResult::NoMatch;
}

} // namespace RaftRuntime::Zenoh
