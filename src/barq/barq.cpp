/**
 * @file barq.cpp
 * @brief BARQ (Burst Access Reader Queue) Implementation - Ultra-Fast "Shoot and Forget"
 */

#include "barq.hpp"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

// Non-temporal store intrinsics
#if defined(__x86_64__) || defined(_M_X64)
#include <emmintrin.h>  // SSE2
#define HAS_NT_STORE 1
#else
#define HAS_NT_STORE 0
#endif

namespace BARQ {

// ============================================================================
// Utility Functions
// ============================================================================

static inline size_t alignUp(size_t value, size_t alignment) {
    return (value + alignment - 1) & ~(alignment - 1);
}

// Non-temporal memcpy for large data (bypasses cache)
static void ntMemcpy(void* dst, const void* src, size_t size) {
#if HAS_NT_STORE
    // Use non-temporal stores for cache bypass
    auto* d = static_cast<__m128i*>(dst);
    const auto* s = static_cast<const __m128i*>(src);
    
    size_t chunks = size / 16;
    size_t remainder = size % 16;
    
    for (size_t i = 0; i < chunks; i++) {
        _mm_stream_si128(d + i, _mm_loadu_si128(s + i));
    }
    
    // Handle remainder
    if (remainder > 0) {
        std::memcpy(
            static_cast<char*>(dst) + chunks * 16,
            static_cast<const char*>(src) + chunks * 16,
            remainder
        );
    }
    
    // Ensure all stores are visible
    _mm_sfence();
#else
    std::memcpy(dst, src, size);
#endif
}

// ============================================================================
// Writer Implementation
// ============================================================================

Writer::Writer(const std::string& name, size_t max_size, bool use_huge_pages)
    : name_(name)
    , max_size_(max_size)
    , use_huge_pages_(use_huge_pages)
    , initialized_(false)
    , huge_pages_active_(false)
    , fd_(-1)
    , ptr_(nullptr)
    , shm_size_(0)
    , header_(nullptr)
    , frame_count_(0)
{
    buffer_[0] = nullptr;
    buffer_[1] = nullptr;
}

Writer::~Writer() {
    destroy();
}

bool Writer::init() {
    if (initialized_) return true;
    
    // Calculate sizes
    size_t buffer_size = alignUp(max_size_, CACHE_LINE);
    shm_size_ = sizeof(Header) + buffer_size * 2;
    
    // Align to huge page if using
    if (use_huge_pages_ && shm_size_ >= HUGE_PAGE) {
        shm_size_ = alignUp(shm_size_, HUGE_PAGE);
    }
    
    // Remove existing
    shm_unlink(name_.c_str());
    
    // Create SHM
    fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
    if (fd_ < 0) return false;
    
    // Set size
    if (ftruncate(fd_, shm_size_) < 0) {
        close(fd_);
        shm_unlink(name_.c_str());
        return false;
    }
    
    // Map with optimizations
    int flags = MAP_SHARED | MAP_POPULATE;
    
    // Try huge pages first
    if (use_huge_pages_ && shm_size_ >= HUGE_PAGE) {
        ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE,
                    flags | MAP_HUGETLB, fd_, 0);
        if (ptr_ != MAP_FAILED) {
            huge_pages_active_ = true;
        }
    }
    
    // Fallback to regular pages
    if (ptr_ == nullptr || ptr_ == MAP_FAILED) {
        ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, flags, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            close(fd_);
            shm_unlink(name_.c_str());
            ptr_ = nullptr;
            return false;
        }
        huge_pages_active_ = false;
    }
    
    // Lock in RAM (prevent page faults during write)
    mlock(ptr_, shm_size_);
    
    // Advise kernel
    madvise(ptr_, shm_size_, MADV_SEQUENTIAL);
    madvise(ptr_, shm_size_, MADV_WILLNEED);
    
    // Initialize header
    header_ = static_cast<Header*>(ptr_);
    std::memset(header_, 0, sizeof(Header));
    
    header_->magic = MAGIC;
    header_->version = VERSION;
    header_->capacity = max_size_;
    header_->buffer_offset = sizeof(Header);
    header_->flags = huge_pages_active_ ? 1 : 0;
    
    header_->front_idx.store(0, std::memory_order_relaxed);
    header_->seq0.store(0, std::memory_order_relaxed);
    header_->seq1.store(0, std::memory_order_relaxed);
    header_->ts0.store(0, std::memory_order_relaxed);
    header_->ts1.store(0, std::memory_order_relaxed);
    header_->len0.store(0, std::memory_order_relaxed);
    header_->len1.store(0, std::memory_order_relaxed);
    header_->heartbeat_ns.store(nowNs(), std::memory_order_relaxed);
    header_->total_writes.store(0, std::memory_order_relaxed);
    header_->total_bytes.store(0, std::memory_order_relaxed);
    
    // Set buffer pointers
    uint8_t* base = static_cast<uint8_t*>(ptr_) + sizeof(Header);
    buffer_[0] = base;
    buffer_[1] = base + buffer_size;
    
    // Memory barrier
    std::atomic_thread_fence(std::memory_order_release);
    
    initialized_ = true;
    return true;
}

bool Writer::write(const void* data, size_t size) {
    if (!initialized_ || size > max_size_) return false;
    
    // Get back buffer
    uint32_t front = header_->front_idx.load(std::memory_order_acquire);
    uint32_t back = 1 - front;
    
    // Copy data - use non-temporal for large data
    if (size >= 4096) {
        ntMemcpy(buffer_[back], data, size);
    } else {
        std::memcpy(buffer_[back], data, size);
    }
    
    // Update metadata
    int64_t now = nowNs();
    ++frame_count_;
    
    if (back == 0) {
        header_->len0.store(size, std::memory_order_relaxed);
        header_->ts0.store(now, std::memory_order_relaxed);
        header_->seq0.store(frame_count_, std::memory_order_relaxed);
    } else {
        header_->len1.store(size, std::memory_order_relaxed);
        header_->ts1.store(now, std::memory_order_relaxed);
        header_->seq1.store(frame_count_, std::memory_order_relaxed);
    }
    
    header_->heartbeat_ns.store(now, std::memory_order_relaxed);
    
    // Atomic swap with release semantics
    header_->front_idx.store(back, std::memory_order_release);
    
    return true;
}

void* Writer::getWriteBuffer() {
    if (!initialized_) return nullptr;
    uint32_t front = header_->front_idx.load(std::memory_order_acquire);
    return buffer_[1 - front];
}

bool Writer::commit(size_t size) {
    if (!initialized_ || size > max_size_) return false;
    
    uint32_t front = header_->front_idx.load(std::memory_order_acquire);
    uint32_t back = 1 - front;
    
    int64_t now = nowNs();
    ++frame_count_;
    
    if (back == 0) {
        header_->len0.store(size, std::memory_order_relaxed);
        header_->ts0.store(now, std::memory_order_relaxed);
        header_->seq0.store(frame_count_, std::memory_order_relaxed);
    } else {
        header_->len1.store(size, std::memory_order_relaxed);
        header_->ts1.store(now, std::memory_order_relaxed);
        header_->seq1.store(frame_count_, std::memory_order_relaxed);
    }
    
    header_->heartbeat_ns.store(now, std::memory_order_relaxed);
    header_->total_writes.fetch_add(1, std::memory_order_relaxed);
    header_->total_bytes.fetch_add(size, std::memory_order_relaxed);
    
    header_->front_idx.store(back, std::memory_order_release);
    
    return true;
}

void Writer::destroy() {
    if (ptr_ && ptr_ != MAP_FAILED) {
        munmap(ptr_, shm_size_);
        ptr_ = nullptr;
    }
    if (fd_ >= 0) {
        close(fd_);
        shm_unlink(name_.c_str());
        fd_ = -1;
    }
    header_ = nullptr;
    buffer_[0] = nullptr;
    buffer_[1] = nullptr;
    initialized_ = false;
}

int64_t Writer::nowNs() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

// ============================================================================
// Reader Implementation
// ============================================================================

Reader::Reader(const std::string& name, size_t max_size)
    : name_(name)
    , max_size_(max_size)
    , initialized_(false)
    , fd_(-1)
    , ptr_(nullptr)
    , shm_size_(0)
    , header_(nullptr)
    , last_seq_(0)
    , dropped_(0)
{
    buffer_[0] = nullptr;
    buffer_[1] = nullptr;
}

Reader::~Reader() {
    if (ptr_ && ptr_ != MAP_FAILED) {
        munmap(ptr_, shm_size_);
    }
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool Reader::init() {
    if (initialized_) return true;
    
    // Open existing SHM
    fd_ = shm_open(name_.c_str(), O_RDONLY, 0666);
    if (fd_ < 0) return false;
    
    // Get size
    struct stat st;
    if (fstat(fd_, &st) < 0) {
        close(fd_);
        fd_ = -1;
        return false;
    }
    shm_size_ = st.st_size;
    
    // Map (try huge pages if they were used)
    ptr_ = mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, fd_, 0);
    if (ptr_ == MAP_FAILED) {
        close(fd_);
        fd_ = -1;
        ptr_ = nullptr;
        return false;
    }
    
    // Validate header
    header_ = static_cast<Header*>(ptr_);
    if (header_->magic != MAGIC) {
        munmap(ptr_, shm_size_);
        close(fd_);
        ptr_ = nullptr;
        fd_ = -1;
        return false;
    }
    
    // Set buffer pointers
    size_t buffer_size = alignUp(max_size_, CACHE_LINE);
    const uint8_t* base = static_cast<const uint8_t*>(ptr_) + sizeof(Header);
    buffer_[0] = base;
    buffer_[1] = base + buffer_size;
    
    // Advise kernel for read pattern
    madvise(ptr_, shm_size_, MADV_SEQUENTIAL);
    madvise(ptr_, shm_size_, MADV_WILLNEED);
    
    initialized_ = true;
    return true;
}

const void* Reader::getLatest(size_t& size, int64_t& timestamp_ns) {
    if (!initialized_) return nullptr;
    
    // Load front index with acquire semantics
    uint32_t front = header_->front_idx.load(std::memory_order_acquire);
    
    // Get sequence and metadata
    uint64_t seq;
    size_t len;
    int64_t ts;
    
    if (front == 0) {
        seq = header_->seq0.load(std::memory_order_relaxed);
        len = header_->len0.load(std::memory_order_relaxed);
        ts = header_->ts0.load(std::memory_order_relaxed);
    } else {
        seq = header_->seq1.load(std::memory_order_relaxed);
        len = header_->len1.load(std::memory_order_relaxed);
        ts = header_->ts1.load(std::memory_order_relaxed);
    }
    
    // Check for new data
    if (seq == last_seq_) {
        return nullptr;  // No new data
    }
    
    // Track dropped frames
    if (last_seq_ > 0 && seq > last_seq_ + 1) {
        dropped_ += (seq - last_seq_ - 1);
    }
    
    last_seq_ = seq;
    size = len;
    timestamp_ns = ts;
    
    // Return pointer directly to SHM - true zero-copy!
    return buffer_[front];
}

bool Reader::isWriterAlive(uint32_t timeout_ms) const {
    if (!initialized_) return false;
    
    int64_t hb = header_->heartbeat_ns.load(std::memory_order_relaxed);
    int64_t now = nowNs();
    int64_t diff_ms = (now - hb) / 1000000;
    
    return diff_ms < static_cast<int64_t>(timeout_ms);
}

int64_t Reader::nowNs() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

} // namespace BARQ
