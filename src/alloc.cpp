#include "alloc.hpp"
#include <cstdlib>
#include <sys/mman.h>
#include <cstdint>

namespace mem {
constexpr size_t PAGE_SIZE = 4 * 1024;
auto
GeneralPurposeAllocator::alloc(size_t len) -> void*
{
  return std::malloc(len);
}
auto
GeneralPurposeAllocator::alignedAlloc(size_t len, size_t alignment) -> void*
{
  return std::aligned_alloc(alignment, len);
}
auto
GeneralPurposeAllocator::free(void* p) -> void
{
  return std::free(p);
}
auto PageAllocator::alloc(size_t len) -> void* {
  return 0;
}
auto PageAllocator::free(void* p) -> void {
  return;
}
struct ArenaAllocator::Impl {
  struct PageList {
    struct PageList* next;
  };
  struct PageList* arena_head;
  size_t page_offset;
};
auto
ArenaAllocator::alloc(size_t len) -> void*
{
  // using PageList = ArenaAllocator::Impl::PageList;
  if (not m_impl) return nullptr;
  // FIXME: What if len > PAGE_SIZE?
  if (not m_impl->arena_head or (PAGE_SIZE - m_impl->page_offset) < len) {
    void* new_mem = mmap(
      0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    struct ArenaAllocator::Impl::PageList* new_page =
      reinterpret_cast<struct ArenaAllocator::Impl::PageList*>(new_mem);
    struct ArenaAllocator::Impl::PageList* old_head = m_impl->arena_head;
    new_page->next = m_impl->arena_head;
    m_impl->arena_head = new_page;
    m_impl->page_offset = sizeof(struct ArenaAllocator::Impl::PageList);
  }
  uint8_t *ptr = (uint8_t*) m_impl->arena_head + m_impl->page_offset;
  m_impl->page_offset += len;
  return (void*) ptr;
}
auto
ArenaAllocator::free(void* p) -> void
{
  // TODO: There is an optimization that allows the top of the stack to be freed
  return;
}
}
