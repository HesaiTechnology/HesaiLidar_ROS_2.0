/**
 * @file barq.hpp
 * @brief BARQ (Burst Access Reader Queue) - Ultra-Fast "Shoot and Forget" Transport
 * 
 * Optimized double-buffer architecture with:
 * - Huge pages support (2MB, auto-fallback)
 * - Cache-line aligned structures (64B)
 * - Non-temporal stores for large writes
 * - Software prefetching
 * - MAP_POPULATE + mlock
 * - Minimal synchronization overhead
 * 
 * "Shoot and Forget" - Writer never waits, reader always gets latest.
 */

#ifndef BARQ_HPP
#define BARQ_HPP

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <string>

namespace BARQ {

// Constants
constexpr uint32_t MAGIC = 0x53484D32;  // "SHM2"
constexpr uint32_t VERSION = 0x00020000;
constexpr size_t CACHE_LINE = 64;
constexpr size_t HUGE_PAGE = 2 * 1024 * 1024;

/**
 * @struct Header
 * @brief Cache-line aligned header for maximum performance
 * 
 * Each critical field has its own cache line to prevent false sharing.
 * Total: 5 cache lines = 320 bytes
 */
struct alignas(CACHE_LINE) Header {
    // === Cache Line 0: Static metadata (64 bytes) ===
    uint32_t magic;
    uint32_t version;
    size_t capacity;
    size_t buffer_offset;      // Offset to buffer A
    uint32_t flags;            // 0x1 = huge pages active
    uint32_t reserved;
    char pad0[CACHE_LINE - 32];
    
    // === Cache Line 1: Front index (64 bytes) ===
    // This is the HOT field - written by writer on every publish
    alignas(CACHE_LINE) std::atomic<uint32_t> front_idx;
    char pad1[CACHE_LINE - sizeof(std::atomic<uint32_t>)];
    
    // === Cache Line 2: Buffer 0 metadata (64 bytes) ===
    alignas(CACHE_LINE) std::atomic<uint64_t> seq0;       // Sequence number
    std::atomic<int64_t> ts0;                              // Timestamp (ns)
    std::atomic<size_t> len0;                              // Data length
    char pad2[CACHE_LINE - 24];
    
    // === Cache Line 3: Buffer 1 metadata (64 bytes) ===
    alignas(CACHE_LINE) std::atomic<uint64_t> seq1;
    std::atomic<int64_t> ts1;
    std::atomic<size_t> len1;
    char pad3[CACHE_LINE - 24];
    
    // === Cache Line 4: Writer state (64 bytes) ===
    alignas(CACHE_LINE) std::atomic<int64_t> heartbeat_ns;
    std::atomic<uint64_t> total_writes;
    std::atomic<uint64_t> total_bytes;
    char pad4[CACHE_LINE - 24];
};

// Verify alignment
static_assert(sizeof(Header) == 5 * CACHE_LINE, "Header must be 5 cache lines");

/**
 * @class Writer
 * @brief Ultra-fast "shoot and forget" writer
 */
class Writer {
public:
    /**
     * @brief Constructor
     * @param name Shared memory name (e.g., "/sensor")
     * @param max_size Maximum data size per write
     * @param use_huge_pages Try to use 2MB huge pages
     */
    Writer(const std::string& name, size_t max_size, bool use_huge_pages = true);
    ~Writer();
    
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    
    /**
     * @brief Initialize shared memory
     * @return true on success
     */
    bool init();
    
    /**
     * @brief Write data (shoot and forget)
     * 
     * Uses non-temporal stores for data > 4KB.
     * Never blocks, always succeeds if initialized.
     * 
     * @param data Pointer to data
     * @param size Size in bytes
     * @return true on success
     */
    bool write(const void* data, size_t size);
    
    /**
     * @brief Get write buffer for zero-copy writing
     * Call commit() after filling.
     * @return Pointer to back buffer
     */
    void* getWriteBuffer();
    
    /**
     * @brief Commit zero-copy write
     * @param size Actual size written
     * @return true on success
     */
    bool commit(size_t size);
    
    /**
     * @brief Check if ready
     */
    bool isReady() const { return initialized_; }
    
    /**
     * @brief Get frame count
     */
    uint64_t getFrameCount() const { return frame_count_; }
    
    /**
     * @brief Clean up
     */
    void destroy();

private:
    std::string name_;
    size_t max_size_;
    bool use_huge_pages_;
    bool initialized_;
    bool huge_pages_active_;
    
    int fd_;
    void* ptr_;
    size_t shm_size_;
    
    Header* header_;
    uint8_t* buffer_[2];
    uint64_t frame_count_;
    
    void writeNonTemporal(void* dst, const void* src, size_t size);
    int64_t nowNs();
};

/**
 * @class Reader
 * @brief Ultra-fast zero-copy reader
 */
class Reader {
public:
    /**
     * @brief Constructor
     * @param name Shared memory name
     * @param max_size Maximum expected data size
     */
    Reader(const std::string& name, size_t max_size);
    ~Reader();
    
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    
    /**
     * @brief Connect to writer's shared memory
     * @return true on success
     */
    bool init();
    
    /**
     * @brief Get pointer to latest data (true zero-copy)
     * 
     * Returns pointer directly into shared memory.
     * Valid until next call to getLatest().
     * 
     * @param size Output: size of data
     * @param timestamp_ns Output: timestamp when written
     * @return Pointer to data, nullptr if no new data
     */
    const void* getLatest(size_t& size, int64_t& timestamp_ns);
    
    /**
     * @brief Check if writer is alive
     * @param timeout_ms Timeout in milliseconds
     * @return true if writer heartbeat is recent
     */
    bool isWriterAlive(uint32_t timeout_ms = 1000) const;
    
    /**
     * @brief Check if ready
     */
    bool isReady() const { return initialized_; }
    
    /**
     * @brief Get number of dropped frames
     */
    uint64_t getDropped() const { return dropped_; }
    
    /**
     * @brief Get last sequence number
     */
    uint64_t getLastSeq() const { return last_seq_; }

private:
    std::string name_;
    size_t max_size_;
    bool initialized_;
    
    int fd_;
    void* ptr_;
    size_t shm_size_;
    
    Header* header_;
    const uint8_t* buffer_[2];
    
    uint64_t last_seq_;
    uint64_t dropped_;
    
    int64_t nowNs() const;
};

} // namespace BARQ

#endif // BARQ_HPP
