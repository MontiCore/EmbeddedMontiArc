#include "caching.h"




ulong FifoCache::read( ulong address, uint sec_id ) {
    auto &tags = *section_tags[sec_id];
    auto local_index = tags.range.get_local_index( address ) / block_size;
    auto &tag = tags.bit_mask[local_index];
    if ( tag )
        return read_time;
        
    //Cache miss: register tag
    tag = true;
    ulong old;
    if ( fifo_buffer.push( to_key( local_index, sec_id ), old ) ) {
        //Must un-tag old address
        section_tags[get_sec_id( old )]->bit_mask[get_local_index( old )] = false;
    }
    return read_time + underlying_memory->read( address, sec_id );
}
ulong FifoCache::write( ulong address, uint sec_id ) {
    auto &tags = *section_tags[sec_id];
    auto local_index = tags.range.get_local_index( address ) / block_size;
    auto &tag = tags.bit_mask[local_index];
    if ( tag )
        return write_time;
        
    //Cache miss: register tag
    tag = true;
    ulong old;
    if ( fifo_buffer.push( to_key( local_index, sec_id ), old ) ) {
        //Must un-tag old address
        section_tags[get_sec_id( old )]->bit_mask[get_local_index( old )] = false;
    }
    underlying_memory->write( address, sec_id );
    return write_time;
}