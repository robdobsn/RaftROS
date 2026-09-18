#include <cstddef>
#include <new>

void* operator new(std::size_t, const std::nothrow_t&) noexcept
{
    return nullptr;
}