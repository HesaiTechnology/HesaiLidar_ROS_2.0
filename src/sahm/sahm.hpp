/**
 * @file sahm.hpp
 * @brief SAHM - Sensor Acquisition to Host Memory with Ring Buffer
 * 
 * Architecture:
 * - Each reader has a ring buffer of N slots
 * - Writer writes cyclically to slots
 * - Reader copies to history when it wants
 * - Zero synchronization overhead per write
 */

#ifndef SAHM_HPP
#define SAHM_HPP

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <string>
#include <vector>
#include <memory>

namespace SAHM {

// Constants
constexpr uint32_t DIRECT_MAGIC = 0xD1EC7002;  // v2
constexpr size_t MAX_READERS = 16;
constexpr size_t SHM_NAME_LEN = 64;
constexpr size_t CACHE_LINE = 64;
constexpr size_t DEFAULT_RING_SIZE = 30;

/**
 * @struct ControlHeader
 * @brief Shared control channel for reader registration
 */
struct alignas(CACHE_LINE) ControlHeader {
    uint32_t magic;
    uint32_t version;
    size_t max_slot_size;           // Max data per slot
    
    std::atomic<uint32_t> num_readers;
    std::atomic<int64_t> writer_heartbeat_ns;
    
    // Reader registration
    char reader_shm_names[MAX_READERS][SHM_NAME_LEN];
    std::atomic<bool> reader_active[MAX_READERS];
    uint32_t reader_ring_sizes[MAX_READERS];
};

/**
 * @struct RingSlot
 * @brief Single slot in the ring buffer
 */
struct alignas(CACHE_LINE) RingSlot {
    std::atomic<uint64_t> sequence;     // Monotonic counter
    std::atomic<int64_t> timestamp_ns;  // When written
    std::atomic<size_t> data_size;      // Actual data size
    // data[] follows immediately after (flexible array)
};

/**
 * @struct RingBufferHeader
 * @brief Header for the ring buffer in reader's SHM
 */
struct alignas(CACHE_LINE) RingBufferHeader {
    uint32_t magic;
    uint32_t ring_size;                 // Number of slots
    size_t slot_data_size;              // Max data per slot
    size_t slot_total_size;             // sizeof(RingSlot) + slot_data_size
    
    std::atomic<uint32_t> write_idx;    // Next slot to write (cyclic)
    std::atomic<uint64_t> total_writes; // Total writes (monotonic)
    
    // Slots start after header at offset sizeof(RingBufferHeader)
};

/**
 * @class DirectWriter
 * @brief Writer that pushes data to reader ring buffers
 */
class DirectWriter {
public:
    DirectWriter(const std::string& channel_name, size_t max_slot_size);
    ~DirectWriter();
    
    DirectWriter(const DirectWriter&) = delete;
    DirectWriter& operator=(const DirectWriter&) = delete;
    
    /**
     * @brief Initialize control channel
     */
    bool init();
    
    /**
     * @brief Write data to next slot in all reader ring buffers
     * @return Number of readers written to
     */
    int write(const void* data, size_t size);
    
    /**
     * @brief Get direct pointers to current write slots
     * @return Vector of pointers to slot data areas
     */
    std::vector<void*> getWriteSlots();
    
    /**
     * @brief Commit after writing directly to slots
     */
    int commitSlots(size_t size);
    
    bool isReady() const { return is_initialized_; }
    uint32_t getReaderCount() const;
    void destroy();

private:
    void discoverReaders();
    int64_t getCurrentTimestampNs();
    
    std::string channel_name_;
    size_t max_slot_size_;
    bool is_initialized_;
    
    int control_fd_;
    void* control_ptr_;
    size_t control_size_;
    ControlHeader* header_;
    
    struct ReaderInfo {
        int fd;
        void* ptr;
        size_t size;
        RingBufferHeader* ring_header;
        uint8_t* slots_base;  // Pointer to first slot
        uint32_t ring_size;
        bool valid;
    };
    std::vector<ReaderInfo> readers_;
};

/**
 * @class DirectReader
 * @brief Reader with ring buffer for incoming data
 */
class DirectReader {
public:
    /**
     * @brief Constructor
     * @param channel_name Control channel name
     * @param max_slot_size Maximum data size per slot
     * @param ring_size Number of slots in ring buffer (default 30)
     */
    DirectReader(const std::string& channel_name, 
                 size_t max_slot_size,
                 uint32_t ring_size = DEFAULT_RING_SIZE);
    ~DirectReader();
    
    DirectReader(const DirectReader&) = delete;
    DirectReader& operator=(const DirectReader&) = delete;
    
    /**
     * @brief Initialize and register with control channel
     */
    bool init();
    
    /**
     * @brief Get pointer to latest written slot (no copy!)
     * @param size Output: size of data in slot
     * @return Pointer to data, or nullptr if no data yet
     */
    const void* getLatest(size_t& size);
    
    /**
     * @brief Get pointer to specific slot by index
     * @param slot_idx Index 0 to ring_size-1
     * @param size Output: size of data in slot
     * @return Pointer to data, or nullptr
     */
    const void* getSlot(uint32_t slot_idx, size_t& size);
    
    /**
     * @brief Get total number of writes since start
     */
    uint64_t getTotalWrites() const;
    
    /**
     * @brief Get current write index
     */
    uint32_t getWriteIndex() const;
    
    /**
     * @brief Get timestamp of latest slot
     */
    int64_t getLatestTimestampNs() const;
    
    /**
     * @brief Get timestamp of specific slot
     */
    int64_t getSlotTimestampNs(uint32_t slot_idx) const;
    
    /**
     * @brief Get sequence number of slot
     */
    uint64_t getSlotSequence(uint32_t slot_idx) const;
    
    /**
     * @brief Check if writer is alive
     */
    bool isWriterAlive(uint32_t timeout_ms = 1000) const;
    
    bool isReady() const { return is_initialized_; }
    uint32_t getRingSize() const { return ring_size_; }

private:
    RingSlot* getSlotPtr(uint32_t idx);
    
    std::string channel_name_;
    std::string my_shm_name_;
    size_t max_slot_size_;
    uint32_t ring_size_;
    bool is_initialized_;
    int my_slot_idx_;  // Slot in control channel
    
    // Control channel
    int control_fd_;
    void* control_ptr_;
    ControlHeader* header_;
    
    // My ring buffer
    int buffer_fd_;
    void* buffer_ptr_;
    size_t buffer_size_;
    RingBufferHeader* ring_header_;
    uint8_t* slots_base_;
    size_t slot_total_size_;
};

} // namespace SAHM

#endif // SAHM_HPP
