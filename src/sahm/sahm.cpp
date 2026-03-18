/**
 * @file sahm.cpp
 * @brief SAHM v2 - Sensor In Memory with Ring Buffer
 */

#include "sahm/sahm.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

namespace SAHM {

// ============================================================================
// DirectWriter Implementation
// ============================================================================

DirectWriter::DirectWriter(const std::string& channel_name, size_t max_slot_size)
    : channel_name_(channel_name)
    , max_slot_size_(max_slot_size)
    , is_initialized_(false)
    , control_fd_(-1)
    , control_ptr_(nullptr)
    , control_size_(sizeof(ControlHeader))
    , header_(nullptr)
{
}

DirectWriter::~DirectWriter() {
    destroy();
}

int64_t DirectWriter::getCurrentTimestampNs() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

bool DirectWriter::init() {
    if (is_initialized_) return true;
    
    // Create control channel
    shm_unlink(channel_name_.c_str());
    control_fd_ = shm_open(channel_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (control_fd_ < 0) return false;
    
    if (ftruncate(control_fd_, control_size_) < 0) {
        close(control_fd_);
        shm_unlink(channel_name_.c_str());
        return false;
    }
    
    control_ptr_ = mmap(nullptr, control_size_, PROT_READ | PROT_WRITE,
                        MAP_SHARED, control_fd_, 0);
    if (control_ptr_ == MAP_FAILED) {
        close(control_fd_);
        shm_unlink(channel_name_.c_str());
        return false;
    }
    
    // Initialize header
    header_ = static_cast<ControlHeader*>(control_ptr_);
    std::memset(header_, 0, sizeof(ControlHeader));
    header_->magic = DIRECT_MAGIC;
    header_->version = 2;
    header_->max_slot_size = max_slot_size_;
    header_->num_readers.store(0);
    header_->writer_heartbeat_ns.store(getCurrentTimestampNs());
    
    for (size_t i = 0; i < MAX_READERS; ++i) {
        header_->reader_active[i].store(false);
        header_->reader_shm_names[i][0] = '\0';
        header_->reader_ring_sizes[i] = 0;
    }
    
    readers_.resize(MAX_READERS);
    for (auto& r : readers_) {
        r.fd = -1;
        r.ptr = nullptr;
        r.ring_header = nullptr;
        r.slots_base = nullptr;
        r.valid = false;
    }
    
    is_initialized_ = true;
    return true;
}

void DirectWriter::discoverReaders() {
    if (!header_) return;
    
    for (size_t i = 0; i < MAX_READERS; ++i) {
        bool active = header_->reader_active[i].load(std::memory_order_acquire);
        
        if (active && !readers_[i].valid) {
            const char* name = header_->reader_shm_names[i];
            if (name[0] == '\0') continue;
            
            int fd = shm_open(name, O_RDWR, 0666);
            if (fd < 0) continue;
            
            // Calculate buffer size
            uint32_t ring_size = header_->reader_ring_sizes[i];
            if (ring_size == 0) ring_size = DEFAULT_RING_SIZE;
            
            size_t slot_total = sizeof(RingSlot) + max_slot_size_;
            size_t buf_size = sizeof(RingBufferHeader) + ring_size * slot_total;
            
            void* ptr = mmap(nullptr, buf_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, 0);
            if (ptr == MAP_FAILED) {
                close(fd);
                continue;
            }
            
            readers_[i].fd = fd;
            readers_[i].ptr = ptr;
            readers_[i].size = buf_size;
            readers_[i].ring_header = static_cast<RingBufferHeader*>(ptr);
            readers_[i].slots_base = static_cast<uint8_t*>(ptr) + sizeof(RingBufferHeader);
            readers_[i].ring_size = ring_size;
            readers_[i].valid = true;
        }
        else if (!active && readers_[i].valid) {
            munmap(readers_[i].ptr, readers_[i].size);
            close(readers_[i].fd);
            readers_[i].valid = false;
        }
    }
}

int DirectWriter::write(const void* data, size_t size) {
    if (!is_initialized_ || size > max_slot_size_) return 0;
    
    discoverReaders();
    
    int64_t timestamp = getCurrentTimestampNs();
    int written = 0;
    
    for (size_t i = 0; i < MAX_READERS; ++i) {
        if (!readers_[i].valid) continue;
        
        RingBufferHeader* rh = readers_[i].ring_header;
        uint32_t ring_size = readers_[i].ring_size;
        size_t slot_total = rh->slot_total_size;
        
        // Get current write slot
        uint32_t idx = rh->write_idx.load(std::memory_order_relaxed);
        uint8_t* slot_ptr = readers_[i].slots_base + idx * slot_total;
        RingSlot* slot = reinterpret_cast<RingSlot*>(slot_ptr);
        uint8_t* slot_data = slot_ptr + sizeof(RingSlot);
        
        // Write data
        std::memcpy(slot_data, data, size);
        
        // Update slot metadata
        uint64_t seq = rh->total_writes.load(std::memory_order_relaxed) + 1;
        slot->data_size.store(size, std::memory_order_relaxed);
        slot->timestamp_ns.store(timestamp, std::memory_order_relaxed);
        slot->sequence.store(seq, std::memory_order_release);
        
        // Advance write index (cyclic)
        uint32_t next_idx = (idx + 1) % ring_size;
        rh->write_idx.store(next_idx, std::memory_order_relaxed);
        rh->total_writes.store(seq, std::memory_order_release);
        
        ++written;
    }
    
    header_->writer_heartbeat_ns.store(timestamp, std::memory_order_release);
    return written;
}

std::vector<void*> DirectWriter::getWriteSlots() {
    std::vector<void*> slots;
    
    if (!is_initialized_) return slots;
    
    discoverReaders();
    
    for (size_t i = 0; i < MAX_READERS; ++i) {
        if (!readers_[i].valid) continue;
        
        RingBufferHeader* rh = readers_[i].ring_header;
        uint32_t idx = rh->write_idx.load(std::memory_order_relaxed);
        size_t slot_total = rh->slot_total_size;
        
        uint8_t* slot_ptr = readers_[i].slots_base + idx * slot_total;
        uint8_t* slot_data = slot_ptr + sizeof(RingSlot);
        
        slots.push_back(slot_data);
    }
    
    return slots;
}

int DirectWriter::commitSlots(size_t size) {
    if (!is_initialized_ || size > max_slot_size_) return 0;
    
    int64_t timestamp = getCurrentTimestampNs();
    int committed = 0;
    
    for (size_t i = 0; i < MAX_READERS; ++i) {
        if (!readers_[i].valid) continue;
        
        RingBufferHeader* rh = readers_[i].ring_header;
        uint32_t ring_size = readers_[i].ring_size;
        size_t slot_total = rh->slot_total_size;
        
        uint32_t idx = rh->write_idx.load(std::memory_order_relaxed);
        uint8_t* slot_ptr = readers_[i].slots_base + idx * slot_total;
        RingSlot* slot = reinterpret_cast<RingSlot*>(slot_ptr);
        
        uint64_t seq = rh->total_writes.load(std::memory_order_relaxed) + 1;
        slot->data_size.store(size, std::memory_order_relaxed);
        slot->timestamp_ns.store(timestamp, std::memory_order_relaxed);
        slot->sequence.store(seq, std::memory_order_release);
        
        uint32_t next_idx = (idx + 1) % ring_size;
        rh->write_idx.store(next_idx, std::memory_order_relaxed);
        rh->total_writes.store(seq, std::memory_order_release);
        
        ++committed;
    }
    
    header_->writer_heartbeat_ns.store(timestamp, std::memory_order_release);
    return committed;
}

uint32_t DirectWriter::getReaderCount() const {
    if (!header_) return 0;
    return header_->num_readers.load(std::memory_order_relaxed);
}

void DirectWriter::destroy() {
    if (!is_initialized_) return;
    
    for (auto& r : readers_) {
        if (r.valid) {
            munmap(r.ptr, r.size);
            close(r.fd);
        }
    }
    readers_.clear();
    
    if (control_ptr_) {
        munmap(control_ptr_, control_size_);
    }
    if (control_fd_ >= 0) {
        close(control_fd_);
        shm_unlink(channel_name_.c_str());
    }
    
    is_initialized_ = false;
}

// ============================================================================
// DirectReader Implementation
// ============================================================================

DirectReader::DirectReader(const std::string& channel_name, 
                           size_t max_slot_size,
                           uint32_t ring_size)
    : channel_name_(channel_name)
    , max_slot_size_(max_slot_size)
    , ring_size_(ring_size)
    , is_initialized_(false)
    , my_slot_idx_(-1)
    , control_fd_(-1)
    , control_ptr_(nullptr)
    , header_(nullptr)
    , buffer_fd_(-1)
    , buffer_ptr_(nullptr)
    , ring_header_(nullptr)
    , slots_base_(nullptr)
{
    my_shm_name_ = channel_name + "_reader_" + std::to_string(getpid());
    slot_total_size_ = sizeof(RingSlot) + max_slot_size_;
    buffer_size_ = sizeof(RingBufferHeader) + ring_size_ * slot_total_size_;
}

DirectReader::~DirectReader() {
    if (is_initialized_) {
        if (header_ && my_slot_idx_ >= 0) {
            header_->reader_active[my_slot_idx_].store(false, std::memory_order_release);
            header_->num_readers.fetch_sub(1, std::memory_order_relaxed);
        }
        
        if (buffer_ptr_) {
            munmap(buffer_ptr_, buffer_size_);
        }
        if (buffer_fd_ >= 0) {
            close(buffer_fd_);
            shm_unlink(my_shm_name_.c_str());
        }
        
        if (control_ptr_) {
            munmap(control_ptr_, sizeof(ControlHeader));
        }
        if (control_fd_ >= 0) {
            close(control_fd_);
        }
    }
}

bool DirectReader::init() {
    if (is_initialized_) return true;
    
    // Open control channel
    control_fd_ = shm_open(channel_name_.c_str(), O_RDWR, 0666);
    if (control_fd_ < 0) return false;
    
    control_ptr_ = mmap(nullptr, sizeof(ControlHeader), PROT_READ | PROT_WRITE,
                        MAP_SHARED, control_fd_, 0);
    if (control_ptr_ == MAP_FAILED) {
        close(control_fd_);
        return false;
    }
    
    header_ = static_cast<ControlHeader*>(control_ptr_);
    
    if (header_->magic != DIRECT_MAGIC) {
        munmap(control_ptr_, sizeof(ControlHeader));
        close(control_fd_);
        return false;
    }
    
    // Create ring buffer SHM
    shm_unlink(my_shm_name_.c_str());
    buffer_fd_ = shm_open(my_shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (buffer_fd_ < 0) {
        munmap(control_ptr_, sizeof(ControlHeader));
        close(control_fd_);
        return false;
    }
    
    if (ftruncate(buffer_fd_, buffer_size_) < 0) {
        close(buffer_fd_);
        shm_unlink(my_shm_name_.c_str());
        munmap(control_ptr_, sizeof(ControlHeader));
        close(control_fd_);
        return false;
    }
    
    buffer_ptr_ = mmap(nullptr, buffer_size_, PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_POPULATE, buffer_fd_, 0);
    if (buffer_ptr_ == MAP_FAILED) {
        close(buffer_fd_);
        shm_unlink(my_shm_name_.c_str());
        munmap(control_ptr_, sizeof(ControlHeader));
        close(control_fd_);
        return false;
    }
    
    mlock(buffer_ptr_, buffer_size_);
    
    // Initialize ring buffer header
    ring_header_ = static_cast<RingBufferHeader*>(buffer_ptr_);
    ring_header_->magic = DIRECT_MAGIC;
    ring_header_->ring_size = ring_size_;
    ring_header_->slot_data_size = max_slot_size_;
    ring_header_->slot_total_size = slot_total_size_;
    ring_header_->write_idx.store(0);
    ring_header_->total_writes.store(0);
    
    slots_base_ = static_cast<uint8_t*>(buffer_ptr_) + sizeof(RingBufferHeader);
    
    // Initialize all slots
    for (uint32_t i = 0; i < ring_size_; ++i) {
        RingSlot* slot = getSlotPtr(i);
        slot->sequence.store(0);
        slot->timestamp_ns.store(0);
        slot->data_size.store(0);
    }
    
    // Register with control channel
    for (size_t i = 0; i < MAX_READERS; ++i) {
        bool expected = false;
        if (header_->reader_active[i].compare_exchange_strong(expected, true)) {
            my_slot_idx_ = i;
            std::strncpy(header_->reader_shm_names[i], my_shm_name_.c_str(), SHM_NAME_LEN - 1);
            header_->reader_shm_names[i][SHM_NAME_LEN - 1] = '\0';
            header_->reader_ring_sizes[i] = ring_size_;
            header_->num_readers.fetch_add(1, std::memory_order_relaxed);
            break;
        }
    }
    
    if (my_slot_idx_ < 0) {
        munmap(buffer_ptr_, buffer_size_);
        close(buffer_fd_);
        shm_unlink(my_shm_name_.c_str());
        munmap(control_ptr_, sizeof(ControlHeader));
        close(control_fd_);
        return false;
    }
    
    is_initialized_ = true;
    return true;
}

RingSlot* DirectReader::getSlotPtr(uint32_t idx) {
    if (idx >= ring_size_) return nullptr;
    return reinterpret_cast<RingSlot*>(slots_base_ + idx * slot_total_size_);
}

const void* DirectReader::getLatest(size_t& size) {
    if (!is_initialized_) return nullptr;
    
    uint64_t total = ring_header_->total_writes.load(std::memory_order_acquire);
    if (total == 0) return nullptr;
    
    // Latest is one before current write_idx
    uint32_t write_idx = ring_header_->write_idx.load(std::memory_order_relaxed);
    uint32_t latest_idx = (write_idx + ring_size_ - 1) % ring_size_;
    
    RingSlot* slot = getSlotPtr(latest_idx);
    size = slot->data_size.load(std::memory_order_relaxed);
    
    return slots_base_ + latest_idx * slot_total_size_ + sizeof(RingSlot);
}

const void* DirectReader::getSlot(uint32_t slot_idx, size_t& size) {
    if (!is_initialized_ || slot_idx >= ring_size_) return nullptr;
    
    RingSlot* slot = getSlotPtr(slot_idx);
    if (slot->sequence.load(std::memory_order_acquire) == 0) return nullptr;
    
    size = slot->data_size.load(std::memory_order_relaxed);
    return slots_base_ + slot_idx * slot_total_size_ + sizeof(RingSlot);
}

uint64_t DirectReader::getTotalWrites() const {
    if (!ring_header_) return 0;
    return ring_header_->total_writes.load(std::memory_order_acquire);
}

uint32_t DirectReader::getWriteIndex() const {
    if (!ring_header_) return 0;
    return ring_header_->write_idx.load(std::memory_order_relaxed);
}

int64_t DirectReader::getLatestTimestampNs() const {
    if (!is_initialized_) return 0;
    
    uint32_t write_idx = ring_header_->write_idx.load(std::memory_order_relaxed);
    uint32_t latest_idx = (write_idx + ring_size_ - 1) % ring_size_;
    
    RingSlot* slot = const_cast<DirectReader*>(this)->getSlotPtr(latest_idx);
    return slot->timestamp_ns.load(std::memory_order_relaxed);
}

int64_t DirectReader::getSlotTimestampNs(uint32_t slot_idx) const {
    if (slot_idx >= ring_size_) return 0;
    RingSlot* slot = const_cast<DirectReader*>(this)->getSlotPtr(slot_idx);
    return slot->timestamp_ns.load(std::memory_order_relaxed);
}

uint64_t DirectReader::getSlotSequence(uint32_t slot_idx) const {
    if (slot_idx >= ring_size_) return 0;
    RingSlot* slot = const_cast<DirectReader*>(this)->getSlotPtr(slot_idx);
    return slot->sequence.load(std::memory_order_acquire);
}

bool DirectReader::isWriterAlive(uint32_t timeout_ms) const {
    if (!header_) return false;
    
    auto now = std::chrono::high_resolution_clock::now();
    int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    int64_t heartbeat = header_->writer_heartbeat_ns.load(std::memory_order_relaxed);
    int64_t diff_ms = (now_ns - heartbeat) / 1000000;
    
    return diff_ms < timeout_ms;
}

} // namespace SAHM
