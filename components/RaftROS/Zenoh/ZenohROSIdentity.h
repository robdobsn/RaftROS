#pragma once

#include "ZenohROSCodec.h"

namespace RaftRuntime::Zenoh
{

// Derives rmw_zenoh-compatible endpoint GIDs: the 128-bit XXH3 hash of the
// endpoint's liveliness token. The hash is a from-specification implementation
// of XXH3 (Yann Collet's xxHash, https://github.com/Cyan4973/xxHash, BSD
// 2-Clause); the prime and default-secret constants below are the fixed
// inputs defined by that specification. No upstream code is included here.
class ZenohROSIdentity
{
public:
    static constexpr size_t TOKEN_BUFFER_SIZE = 8 + 12 + 10 + 32 + 20 + 20 + 2 +
        4 * ZenohROSCodec::MAX_ROS_NAME_SIZE + ZenohROSCodec::MAX_TYPE_NAME_SIZE + 71 +
        ZenohROSCodec::QOS_BUFFER_SIZE;
    using Gid = std::array<uint8_t, ZenohROSCodec::GID_SIZE>;

    static bool deriveEndpointGid(const ZenohROSCodec::NodeIdentity& node,
                                  const ZenohROSCodec::Endpoint& endpoint, Gid& output)
    {
        std::array<char, TOKEN_BUFFER_SIZE> token{};
        if (!ZenohROSCodec::formatEndpointToken(token.data(), token.size(), node, endpoint))
            return false;
        const std::string_view text(token.data());
        if (text.size() <= 16)
            return false;
        const auto* bytes = reinterpret_cast<const uint8_t*>(text.data());
        const auto hash = text.size() <= 240 ? hashMedium(bytes, text.size()) : hashLong(bytes, text.size());
        for (size_t half = 0; half < hash.size(); ++half)
        {
            for (size_t byte = 0; byte < 8; ++byte)
                output[half * 8 + byte] = static_cast<uint8_t>(hash[half] >> (byte * 8));
        }
        return true;
    }

private:
    static constexpr uint64_t PRIME1 = 0x9e3779b185ebca87ULL;
    static constexpr uint64_t PRIME2 = 0xc2b2ae3d27d4eb4fULL;
    static constexpr uint64_t PRIME3 = 0x165667b19e3779f9ULL;
    static constexpr uint64_t PRIME4 = 0x85ebca77c2b2ae63ULL;
    static constexpr uint64_t PRIME5 = 0x27d4eb2f165667c5ULL;
    static constexpr std::array<uint8_t, 192> SECRET = {
        0xb8, 0xfe, 0x6c, 0x39, 0x23, 0xa4, 0x4b, 0xbe,
        0x7c, 0x01, 0x81, 0x2c, 0xf7, 0x21, 0xad, 0x1c,
        0xde, 0xd4, 0x6d, 0xe9, 0x83, 0x90, 0x97, 0xdb,
        0x72, 0x40, 0xa4, 0xa4, 0xb7, 0xb3, 0x67, 0x1f,
        0xcb, 0x79, 0xe6, 0x4e, 0xcc, 0xc0, 0xe5, 0x78,
        0x82, 0x5a, 0xd0, 0x7d, 0xcc, 0xff, 0x72, 0x21,
        0xb8, 0x08, 0x46, 0x74, 0xf7, 0x43, 0x24, 0x8e,
        0xe0, 0x35, 0x90, 0xe6, 0x81, 0x3a, 0x26, 0x4c,
        0x3c, 0x28, 0x52, 0xbb, 0x91, 0xc3, 0x00, 0xcb,
        0x88, 0xd0, 0x65, 0x8b, 0x1b, 0x53, 0x2e, 0xa3,
        0x71, 0x64, 0x48, 0x97, 0xa2, 0x0d, 0xf9, 0x4e,
        0x38, 0x19, 0xef, 0x46, 0xa9, 0xde, 0xac, 0xd8,
        0xa8, 0xfa, 0x76, 0x3f, 0xe3, 0x9c, 0x34, 0x3f,
        0xf9, 0xdc, 0xbb, 0xc7, 0xc7, 0x0b, 0x4f, 0x1d,
        0x8a, 0x51, 0xe0, 0x4b, 0xcd, 0xb4, 0x59, 0x31,
        0xc8, 0x9f, 0x7e, 0xc9, 0xd9, 0x78, 0x73, 0x64,
        0xea, 0xc5, 0xac, 0x83, 0x34, 0xd3, 0xeb, 0xc3,
        0xc5, 0x81, 0xa0, 0xff, 0xfa, 0x13, 0x63, 0xeb,
        0x17, 0x0d, 0xdd, 0x51, 0xb7, 0xf0, 0xda, 0x49,
        0xd3, 0x16, 0x55, 0x26, 0x29, 0xd4, 0x68, 0x9e,
        0x2b, 0x16, 0xbe, 0x58, 0x7d, 0x47, 0xa1, 0xfc,
        0x8f, 0xf8, 0xb8, 0xd1, 0x7a, 0xd0, 0x31, 0xce,
        0x45, 0xcb, 0x3a, 0x8f, 0x95, 0x16, 0x04, 0x28,
        0xaf, 0xd7, 0xfb, 0xca, 0xbb, 0x4b, 0x40, 0x7e
    };

    static uint64_t readWord(const uint8_t* bytes)
    {
        uint64_t value = 0;
        for (size_t offset = 0; offset < 8; ++offset)
            value |= static_cast<uint64_t>(bytes[offset]) << (8 * offset);
        return value;
    }

    static uint64_t multiplyFold(uint64_t left, uint64_t right)
    {
        const uint64_t crossLeft = (left >> 32) * static_cast<uint32_t>(right);
        const uint64_t crossRight = static_cast<uint32_t>(left) * (right >> 32);
        uint64_t high = (left >> 32) * (right >> 32) + (crossLeft >> 32) + (crossRight >> 32);
        uint64_t low = static_cast<uint64_t>(static_cast<uint32_t>(left)) * static_cast<uint32_t>(right);
        const uint64_t first = low;
        low += crossLeft << 32;
        high += low < first;
        const uint64_t second = low;
        low += crossRight << 32;
        high += low < second;
        return low ^ high;
    }

    static uint64_t avalanche(uint64_t value)
    {
        value ^= value >> 37;
        value *= 0x165667919e3779f9ULL;
        return value ^ (value >> 32);
    }

    static void mixPair(std::array<uint64_t, 2>& state, const uint8_t* front,
                        const uint8_t* back, size_t secretOffset)
    {
        const std::array<uint64_t, 4> words = {
            readWord(front), readWord(front + 8), readWord(back), readWord(back + 8)
        };
        for (size_t half = 0; half < state.size(); ++half)
        {
            const size_t word = 2 * half;
            state[half] += multiplyFold(words[word] ^ readWord(SECRET.data() + secretOffset + 8 * word),
                                        words[word + 1] ^ readWord(SECRET.data() + secretOffset + 8 * (word + 1)));
            state[half] ^= words[word ^ 2] + words[(word ^ 2) + 1];
        }
    }

    static std::array<uint64_t, 2> hashMedium(const uint8_t* bytes, size_t length)
    {
        std::array<uint64_t, 2> state{length * PRIME1, 0};
        if (length <= 128)
        {
            size_t pairs = (length + 31) / 32;
            while (pairs > 0)
            {
                --pairs;
                mixPair(state, bytes + 16 * pairs, bytes + length - 16 * (pairs + 1), 32 * pairs);
            }
        }
        else
        {
            for (size_t offset = 0; offset < 128; offset += 32)
                mixPair(state, bytes + offset, bytes + offset + 16, offset);
            for (auto& half : state)
                half = avalanche(half);
            for (size_t offset = 128; offset + 32 <= length; offset += 32)
                mixPair(state, bytes + offset, bytes + offset + 16, offset - 125);
            mixPair(state, bytes + length - 16, bytes + length - 32, 103);
        }
        return {avalanche(state[0] + state[1]),
                uint64_t{0} - avalanche(state[0] * PRIME1 + state[1] * PRIME4 + length * PRIME2)};
    }

    static void accumulateStripe(std::array<uint64_t, 8>& state, const uint8_t* bytes, size_t secretOffset)
    {
        for (size_t lane = 0; lane < state.size(); ++lane)
        {
            const uint64_t data = readWord(bytes + lane * 8);
            const uint64_t keyed = data ^ readWord(SECRET.data() + secretOffset + lane * 8);
            state[lane ^ 1] += data;
            state[lane] += static_cast<uint64_t>(static_cast<uint32_t>(keyed)) * (keyed >> 32);
        }
    }

    static std::array<uint64_t, 2> hashLong(const uint8_t* bytes, size_t length)
    {
        std::array<uint64_t, 8> state = {
            0xc2b2ae3dULL, PRIME1, PRIME2, PRIME3, PRIME4, 0x85ebca77ULL, PRIME5, 0x9e3779b1ULL
        };
        size_t offset = 0;
        size_t stripe = 0;
        while (length - offset > 64)
        {
            accumulateStripe(state, bytes + offset, stripe * 8);
            offset += 64;
            ++stripe;
            if (stripe == 16)
            {
                for (size_t lane = 0; lane < state.size(); ++lane)
                    state[lane] = (state[lane] ^ (state[lane] >> 47) ^ readWord(SECRET.data() + 128 + lane * 8)) * 0x9e3779b1ULL;
                stripe = 0;
            }
        }
        accumulateStripe(state, bytes + length - 64, 121);
        std::array<uint64_t, 2> result{length * PRIME1, ~(length * PRIME2)};
        for (size_t half = 0; half < result.size(); ++half)
        {
            const size_t secretOffset = half == 0 ? 11 : 117;
            for (size_t lane = 0; lane < state.size(); lane += 2)
                result[half] += multiplyFold(state[lane] ^ readWord(SECRET.data() + secretOffset + lane * 8),
                                             state[lane + 1] ^ readWord(SECRET.data() + secretOffset + (lane + 1) * 8));
            result[half] = avalanche(result[half]);
        }
        return result;
    }
};

}