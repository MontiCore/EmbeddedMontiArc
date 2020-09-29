#include "caching.h"




//void FifoCache::set_ticks( ComputerTime &time, uint read_ticks, uint write_ticks ) {
//    read_time = read_ticks * time.cpu_tick_time_pico;
//    write_time = write_ticks * time.cpu_tick_time_pico;
//}
//
//ulong FifoCache::read( ulong address, uint sec_id ) {
//    auto &tags = *section_tags[sec_id];
//    auto local_index = tags.range.get_local_index( address ) / block_size;
//    auto tag = tags.bit_mask[local_index];
//    if ( tag )
//        return read_time;
//        
//    //Cache miss: register tag
//    tag = true;
//    ulong old;
//    if ( fifo_buffer.push( to_key( local_index, sec_id ), old ) ) {
//        //Must un-tag old address
//        section_tags[get_sec_id( old )]->bit_mask[get_local_index( old )] = false;
//    }
//    return read_time + underlying_memory->read( address, sec_id );
//}
//ulong FifoCache::write( ulong address, uint sec_id ) {
//    auto &tags = *section_tags[sec_id];
//    auto local_index = tags.range.get_local_index( address ) / block_size;
//    auto tag = tags.bit_mask[local_index];
//    if ( tag )
//        return write_time;
//        
//    //Cache miss: register tag
//    tag = true;
//    ulong old;
//    if ( fifo_buffer.push( to_key( local_index, sec_id ), old ) ) {
//        //Must un-tag old address
//        section_tags[get_sec_id( old )]->bit_mask[get_local_index( old )] = false;
//    }
//    underlying_memory->write( address, sec_id );
//    return write_time;
//}


NWayAssociativeCache::NWayAssociativeCache(uint N, uint cache_size, uint line_size, ulong read_ticks, ulong write_ticks, ComputerTime& time) :
    N(N), cache_size(cache_size), line_size(line_size), read_time(read_ticks* time.cpu_tick_time_pico), write_time(write_ticks* time.cpu_tick_time_pico)
{
    N_bits = int_log(N);
    cache_size_bits = int_log(cache_size);
    line_size_bits = int_log(line_size);
    // Validate that the sizes are powers of 2:
    if ((1 << N_bits) != N) throw_error("Cache Associativity (N) not a power of 2: " + std::to_string(N));
    if ((1 << line_size_bits) != line_size) throw_error("Cache-Line length not a power of 2: " + std::to_string(line_size));
    if (cache_size % line_size != 0) throw_error("Cache Size not a multiple of the cache-line length (line length: " + std::to_string(line_size)+", cache size: " + std::to_string(cache_size)+")");

    set_bit_offset = line_size_bits;
    set_count = cache_size / N;
    auto set_bits = int_log(set_count);
    set_mask = BIT_MASKS[set_bits];
    tag_bit_offset = set_bit_offset + set_bits;
    tag_mask = (~0LL) ^ BIT_MASKS[tag_bit_offset];
    way_mask = BIT_MASKS[N_bits];
    tags.resize(cache_size);
    for (auto& v : tags) v = 0;
}

ulong NWayAssociativeCache::read(ulong address)
{
    auto set_id = get_set(address);
    auto set = get_tags_buffer(set_id);
    auto tag = get_tag(address);
    auto comp = tag | 1; // First Bit used as "Valid" flag
    // Check for hit
    for (int i = 0; i < N; ++i) {
        if (set[i] == comp) {
            // Cache Hit
            return read_time;
        }
    }

    // Cache miss
    bool full = true;
    for (int i = 0; i < N; ++i) {
        if (!(set[i] & 1)) {
            full = false;
            set[i] = comp;
            return read_time + underlying_memory->read(address);
        }
    }

    // Random eviction (all ways are full)
    auto i = (address>>3) & way_mask;
    set[i] = comp;

    return read_time + underlying_memory->read(address);
}

ulong NWayAssociativeCache::write(ulong address)
{
    auto set_id = get_set(address);
    auto set = get_tags_buffer(set_id);
    auto tag = get_tag(address);
    auto comp = tag | 1; // First Bit used as "Valid" flag
    // Check for hit
    for (int i = 0; i < N; ++i) {
        if (set[i] == comp) {
            // Cache Hit
            return read_time;
        }
    }

    // "Cache Miss" -> Evict
    bool full = true;
    for (int i = 0; i < N; ++i) {
        if (!(set[i] & 1)) {
            full = false;
            set[i] = comp;
            underlying_memory->write(address);
            return write_time;
        }
    }

    // Random eviction (all ways are full)
    auto i = (address >> 3)& way_mask;
    set[i] = comp;

    underlying_memory->write(address);
    return write_time;
}
