/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "caching.h"


NWayAssociativeCache::NWayAssociativeCache(uint level, std::string type, uint N, uint cache_size, uint line_size, ulong read_ticks, ulong write_ticks, ComputerTime& time) :
    level(level), type(type), N(N), cache_size(cache_size), line_size(line_size), computer_time(time), read_time(read_ticks* time.cpu_tick_time_pico), write_time(write_ticks* time.cpu_tick_time_pico)
{

    // to resolve memory address to cache address
    N_bits = bit_count(N);
    line_size_bits = bit_count(line_size);
    cache_line_count = cache_size / line_size ;
    set_count = cache_line_count / N;
    set_bits = bit_count(set_count);
    tag_bits = cpu_bit - set_bits - line_size_bits;

    set_bit_offset = line_size_bits;
    tag_bit_offset = set_bit_offset + set_bits;
    set_mask = BIT_MASKS[set_bits];
    offset_mask = BIT_MASKS[line_size_bits];
    tag_mask = BIT_MASKS[tag_bits];

    // Validate that the sizes are powers of 2:
    if ((1 << N_bits) != N) throw_error("Cache Associativity (N) not a power of 2: " + std::to_string(N));
    if ((1 << line_size_bits) != line_size) throw_error("Cache-Line length not a power of 2: " + std::to_string(line_size));
    if (this->cache_size % line_size != 0) throw_error("Cache Size not a multiple of the cache-line length (line length: " + std::to_string(line_size)+", cache size: " + std::to_string(this->cache_size)+")");
    if ( cache_line_count % N != 0 ) throw_error("Number of Cache Lines not a multiple of N (number of cache lines: " + std::to_string(cache_line_count) + ", N: " + std::to_string(N));
    if ( !is_power_of_two(set_count) ) throw_error("Number of set count not power of 2: " + std::to_string(set_count));

    tags.resize(this->cache_size);
    for (auto& v : tags) v = 0;

    lru_queue.resize(set_count);
    for (auto& set_lru_que : lru_queue) {
        set_lru_que.resize(N);
        for (int i = 0 ; i < N ; ++i) {
            set_lru_que[i] = i;
        }
    }

    read_accesses = 0;
    write_accesses = 0;
    read_hits = 0;
    write_hits = 0;
    read_misses = 0;
    write_misses = 0;

    last_access_result = -1;
}

ulong NWayAssociativeCache::read(ulong address)
{
    read_accesses += 1;
    auto set_id = get_set_idx(address);
    auto offset_in_cache_line = get_offset(address);

    if (offset_in_cache_line > line_size) throw_error("offset bigger than cache line size (offset: " + std::to_string(offset_in_cache_line) + ", cache line size: " + std::to_string(line_size));
    if (set_id > set_count) throw_error("set index bigger than set count (set index: " + std::to_string(set_id) + ", set count: " + std::to_string(set_count));

    auto set = get_tags_buffer(set_id);
    auto tag = get_tag(address);
    auto tag_valid = (unsigned int long long)pow(2, tag_bits + 1) + tag; // derive a unique key from given memory address
                                                                                // we save '(1 << (tag_bits + 1)) + tag' in a cache byte as tag to compare
                                                                                // just set modify bit to 0 now

    last_access_address = address;
    last_access_set_id = set_id;
    // Check for hit
    for (int i = 0; i < N; ++i) {
        // check only the (cache line begin + offset) where the data of the given memory address could be at
        auto line_offset = i * line_size;
        auto cache_tag = set[line_offset + offset_in_cache_line];
        if (compare_tags( tag_valid, cache_tag )) {
            // Cache Hit
            read_hits += 1;
            last_access_result = 1;

            // write back policy
            // V = 1, M = 0 tag matching, read pending
            // OR V = 1, M = 1, tag matching, read pending
            // --> read data in cache into CPU (= do nothing)

            update_lru_queue(set_id, i);

            return read_time;
        }
    }

    // cache hit in n-th layer --> nth happens. priority goes up
    // cache miss in n-th layer --> search (n+1)-th layer, priority of the missed-cache-line goes up in the n-th layer

    read_misses += 1;
    last_access_result = 0;
    // Cache miss, check for empty lines in the set and load whole cache line
    for (int i = 0; i < N; ++i) {
        // check only the first byte to check if the whole line is full or not, since the minimum amount of data load
        // is not a byte but a cache line
        auto line_offset = i * line_size;
        if ( set[line_offset] == 0) { // if there is a space in this level, there will always be space
            // write back policy
            // V = 0, read/write pending
            // --> read from RAM !
            //     set V = 1, M = 0
            //     GOTO V = 1, M = 0, tag matching, read pending
            //
            // V = 1 , M = 0 read pending
            // --> read data in cache to CPU ( = do nothing)
            load_memory_block(address, set, i, offset_in_cache_line, false); // load whole memory block into the cache line
            update_lru_queue(set_id, i);

            if (typeid(*underlying_memory) == typeid(MemoryTime)) {
                return 66000; // TODO: currently, the DRAM access latency is hardcoded, since the estimated latency calculated based
                              // on memory clock frequency is not accurate
            }
            else {
                return underlying_memory->read(address);
            }
        }
    }

    // apply LRU algorithm to evict Least Recently Used cache-line, when there is no empty cache line
    uint line_idx = get_LRU_line_idx(set_id);
    bool modify_bit = get_modify_bit(*( set + line_idx*line_size + offset_in_cache_line ) );
    if (modify_bit) {
        // V = 1, M = 1, tag not matching, read/write pending
        // --> write cache block to RAM
        //     set V = 0,
        //     GOTO V= 0, read/write pending
        //
        // V = 0, read pending
        // --> read from RAM into cache
        //     set V =1 , M = 0
        //     GOTO V = 1, M = 0, tag matching, read pending
        //
        // V = 1, M = 0, tag matching, read pending
        // --> read date from cache to CPU ( = do nothing)
        load_memory_block(address, set, line_idx, offset_in_cache_line, false);
        update_lru_queue(set_id, line_idx);

        if (typeid(*underlying_memory) == typeid(MemoryTime)) {
            return 66000 + 66000; // read from addr A, write into Addr B
                                  // so twice
        }
        else {
            return 66000 + underlying_memory->read(address);
        }
    }
    else {
        // V = 1, M = 0, tag not matching, read/write pending
        // --> set V = 0
        //     GOTO V = 0, read/write pending
        //
        // V = 0, read,write pending
        // --> read from RAM into cache
        //     set V = 1, M = 0
        //     GOTO V = 1, M = 0, tag matching, read pending
        //
        // V = 1, M = 0, tag matching, read pending
        // --> read data from cache to CPU ( = do nothing)
        load_memory_block(address, set, line_idx, offset_in_cache_line, false);
        update_lru_queue(set_id, line_idx);

        if (typeid(*underlying_memory) == typeid(MemoryTime)) {
            return 66000;
            // the read_time for cache layer is ignored, since it will be very small compared to the memory access time
        }
        else {
            return underlying_memory->read(address);
        }
    }
}

ulong NWayAssociativeCache::write(ulong address)
{
    last_access_address = address;

    write_accesses += 1;
    auto set_id = get_set_idx(address);
    auto offset_in_cache_line = get_offset(address);

    if (offset_in_cache_line > line_size) throw_error("offset bigger than cache line size (offset: " + std::to_string(offset_in_cache_line) + ", cache line size: " + std::to_string(line_size));
    if (set_id > set_count) throw_error("set index bigger than set count (set index: " + std::to_string(set_id) + ", set count: " + std::to_string(set_count));

    auto set = get_tags_buffer(set_id);
    auto tag = get_tag(address);
    auto tag_valid = (unsigned int long long)pow(2, tag_bits + 1) + tag; // derive a unique key from given memory address
                                                                                // we save '(1 << (tag_bits + 1)) + tag' in a cache byte as tag to compare
                                                                                // just set modify bit to 0 now

    last_access_set_id = set_id;
    // Check for hit
    for (int i = 0; i < N; ++i) {
        // check on the offset where the data of the given memory address could be at
        auto line_offset = i * line_size;
        auto cache_tag = * (set + line_offset + offset_in_cache_line );
        if (compare_tags(tag_valid, set[line_offset + offset_in_cache_line]) ) {
            // Cache Hit
            write_hits += 1;
            last_access_result = 1;

            // write back policy
            bool modify_bit = get_modify_bit(cache_tag);
            if(modify_bit) {
                // V = 1, M = 1, tag matching, write pending
                // --> write data from CPU to cache ( = do nothing )
            }
            else {
                // V= 1, M = 0, tag matching, write pending
                // --> write data from CPU to cache ( = do nothing )
                //     set M = 1;
                set_modify_bit(&cache_tag, true);
            }
            update_lru_queue(set_id, i);
            return write_time;
        }
    }

    // cache hit in n-th layer --> nth happens. priority goes up
    // cache miss in n-th layer --> search (n+1)-th layer, priority of the missed-cache-line goes up in the n-th layer

    write_misses += 1;
    last_access_result = 0;
    // Cache miss, check for empty lines in the set and load whole cache line
    for (int i = 0; i < N; ++i) {
        // check only the first byte to check if the whole line is full or not, since the minimum amount of data load
        // is not a byte but a cache line
        auto line_offset = i * line_size;
        if ( set[line_offset] == 0) { // if there is a space in this level, there will always be space

            // write back policy
            // V = 0, write pending
            // --> read data from RAM into cache
            //    set V = 1, M = 0
            //    GOTO V = 1, M = 0, tag matching, write pending
            //
            // V = 1, M = 0, tag matching, write pending
            // --> write data from CPU to cache ( = do nothing)
            //     set M = 1

            // load whole cache line (just indexes acutally)
            load_memory_block(address, set, i, offset_in_cache_line, true); // load whole memory block into the cache line

            update_lru_queue(set_id, i);

            if (typeid(*underlying_memory) == typeid(MemoryTime)) {
                return 66000 + write_time;
            }
            else {
                return write_time + underlying_memory->write(address);
            }
        }
    }

    // apply LRU algorithm, when there is no empty cache line
    uint line_idx = get_LRU_line_idx(set_id);
    //write back policy
    bool modify_bit = get_modify_bit(*( set + line_idx*line_size + offset_in_cache_line ) );
    if (modify_bit) {
        // V = 1, M = 1, tag not matching, write pending
        // --> write data from cache to memory
        //     set V = 0
        //     GOTO V = 0, read/write pending
        //
        // V = 0, read write pending
        // --> read from RAM into cache
        //     set V = 1, M = 0
        //     GOTO V = 1, M = 0, tag matching, write pending
        //
        // V = 1, M = 0, tag matching, write pending
        load_memory_block(address, set, line_idx, offset_in_cache_line, true);
        update_lru_queue(set_id, line_idx);

        if (typeid(*underlying_memory) == typeid(MemoryTime)) {
            return 66000 + 66000 + write_time;
        }
        else {
            return 66000 + write_time + underlying_memory->write(address);
        }
    }
    else {
        // V = 1, M = 0, tag not matching, write pending
        // --> set V = 0
        //     goto V= 0, read/write pending
        //
        // V = 0, read/ write pending
        // --> read from RAM into cache
        //     set V = 1, M = 0
        //     GOTO V = 1, M = 0, tag matching, write pending
        //
        // V = 1, M = 0, tag matching ,write pending
        // --> write data from CPU to cache ( = do nothing )
        //     set M = 1
        load_memory_block(address, set, line_idx, offset_in_cache_line, true);
        update_lru_queue(set_id, line_idx);

        if (typeid(*underlying_memory) == typeid(MemoryTime)) {
            return 66000 + write_time;
        }
        else {
            return write_time + underlying_memory->write(address);
        }
    }
}
