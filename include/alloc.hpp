#pragma once
#include <cstddef>

namespace mem{
class Allocator {
  public:
    virtual auto alloc(size_t len) -> void* = 0;
    virtual auto alignedAlloc(size_t len, size_t alignment)
      -> void* = 0;
    virtual auto free(void* p) -> void = 0;
    virtual ~Allocator() {}
};
class GeneralPurposeAllocator : public Allocator {
  public:
    auto alloc(size_t len) -> void* override; 
    auto alignedAlloc(size_t len, size_t alignment) -> void* override; 
    auto free(void* p) -> void override;
};
class PageAllocator : public Allocator {
  public:
    auto alloc(size_t len) -> void* override;
    auto free(void* p) -> void override;
};
class ArenaAllocator : public Allocator {
  struct Impl;
  struct Impl* m_impl;
  public:
    auto alloc(size_t len) -> void* override;
    auto free(void* p) -> void override;
};
}
