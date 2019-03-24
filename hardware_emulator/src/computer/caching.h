#pragma once

#include "utility.h"
#include "memory.h"
#include "instruction_time.h"





struct FifoCache : public MemoryAccessInterface {
    struct FifoBuffer {
        ulong *data;
        uint start, end;
        uint count;
        uint size;
        FifoBuffer() : count( 0 ), data( nullptr ), size( 0 ), start( 0 ), end( 0 ) {}
        FifoBuffer( Array<ulong> &data ) {
            init( data );
        }
        void init( Array<ulong> &data ) {
            clear();
            this->data = data.begin();
            this->size = data.size();
        }
        void clear() {
            count = 0;
            start = 0;
            end = 0;
        }
        //Adds new_elem, if full pops last to old_elem before and returns true
        bool push( ulong new_elem, ulong &old_elem ) {
            if ( count == size ) { //=> start == end, thus only use and update 'end'
                old_elem = data[end];
                data[end] = new_elem;
                end = ( end + 1 ) % size;
                return true;
            }
            data[start] = new_elem;
            ++count;
            start = ( start + 1 ) % size;
            return false;
        }
    } fifo_buffer;
    Array<ulong> buffer;
    struct SectionTags {
        MemoryRange range;
        Array<bool> bit_mask;
    };
    Array<std::unique_ptr<SectionTags>> section_tags;
    
    ulong read_time;
    ulong write_time;
    uint block_size;
    
    FifoCache( Memory &mem, uint cache_size ) : read_time( 0 ), write_time( 0 ), block_size( 8 ) {
        buffer.init( cache_size );
        fifo_buffer.init( buffer );
        section_tags.init( mem.section_count );
        for ( uint i : urange( mem.section_count ) ) {
            auto tags = new SectionTags();
            auto &sec = *mem.sections[i];
            tags->bit_mask.init( sec.address_range.size / block_size + 1 );
            tags->range = sec.address_range;
            section_tags[i] = std::unique_ptr<SectionTags>( tags );
        }
    }
    
    inline ulong to_key( ulong local_index, ulong sec_id ) {
        return ( sec_id << 32 ) | local_index;
    }
    inline uint get_sec_id( ulong key ) {
        return key >> 32;
    }
    inline uint get_local_index( ulong key ) {
        return ( uint )key;
    }
    ulong read( ulong address, uint sec_id );
    ulong write( ulong address, uint sec_id );
    
};