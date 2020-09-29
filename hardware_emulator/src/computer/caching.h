#pragma once

#include "utility/utility.h"
#include "memory.h"
#include "instruction_time.h"


struct NWayAssociativeCache : public MemoryAccessInterface {

    NWayAssociativeCache(uint N, uint cache_size, uint line_size, ulong read_ticks, ulong write_ticks, ComputerTime& time);

    uint N;
    uint N_bits;
    uint cache_size;
    uint cache_size_bits;
    uint line_size;
    uint line_size_bits;
    uint set_count;
    uint set_bit_offset;
    uint tag_bit_offset;
    ulong set_mask;
    ulong tag_mask;
    ulong way_mask;

    ulong read_time;
    ulong write_time;

    std::vector<ulong> tags;

    ulong read(ulong address);
    ulong write(ulong address);

    inline ulong* get_tags_buffer(int id) {
        return tags.data() + (id * N);
    }

    inline ulong get_set(ulong addr) {
        return (addr >> set_bit_offset) & set_mask;
    }
    inline ulong get_tag(ulong addr) {
        return addr & tag_mask;
    }

    // Invalid for 0
    inline uint int_log(uint val) {
        uint i = 0;
        while (!((val >> i) & 1) && i < 32) ++i;
        return i;
    }
};


/*
    This implementation of the MemoryAccessInterface interface represents a Cache using the
    FIFO replacement policy.
    The cache is initialized with its size expressed in number of blocks it can hold
    The block_size defaults to 8 bytes but can be changed.
    Use set_ticks() to defined the read and write times of the cache (defaults to 0) expressed
    as number of cpu cycles (ticks).

    The cache allocates marking buffers for all sections and holds a fifo queue.
    The marking buffer allow to see is a given address is in the cache without searching
    through the entire queue.

    Another possibility for other cache implementations is having a hashmap with the entries of
    the cache in addition to the structure managing the replacement.
*/
//struct FifoCache : public MemoryAccessInterface {
//    private:
//        struct FifoBuffer {
//            ulong *data;
//            uint start, end;
//            uint count;
//            uint size;
//            FifoBuffer() : count( 0 ), data( nullptr ), size( 0 ), start( 0 ), end( 0 ) {}
//            FifoBuffer(std::vector<ulong> &data ) {
//                init( data );
//            }
//            void init(std::vector<ulong> &data ) {
//                clear();
//                this->data = data.data();
//                this->size = (uint) data.size();
//            }
//            void clear() {
//                count = 0;
//                start = 0;
//                end = 0;
//            }
//            //Adds new_elem, if full pops last to old_elem and returns true
//            bool push( ulong new_elem, ulong &old_elem ) {
//                if ( count == size ) { //=> start == end, thus only use and update 'end'
//                    old_elem = data[end];
//                    data[end] = new_elem;
//                    end = ( end + 1 ) % size;
//                    return true;
//                }
//                data[start] = new_elem;
//                ++count;
//                start = ( start + 1 ) % size;
//                return false;
//            }
//        } fifo_buffer;
//		std::vector<ulong> buffer;
//        struct SectionTags {
//            MemoryRange range;
//			vector_bool bit_mask;
//        };
//		std::vector<std::unique_ptr<SectionTags>> section_tags;
//        
//        
//        inline ulong to_key( ulong local_index, ulong sec_id ) {
//            return ( sec_id << 32 ) | local_index;
//        }
//        inline uint get_sec_id( ulong key ) {
//            return key >> 32;
//        }
//        inline uint get_local_index( ulong key ) {
//            return ( uint )key;
//        }
//        
//        ulong read_time;
//        ulong write_time;
//    public:
//    
//        uint block_size;
//        
//        FifoCache( Memory &mem, uint cache_size ) : read_time( 0 ), write_time( 0 ), block_size( 8 ) {
//            buffer.resize( cache_size );
//            fifo_buffer.init( buffer );
//            section_tags.resize( mem.section_count );
//            for ( uint i : urange( mem.section_count ) ) {
//                auto tags = new SectionTags();
//                auto &sec = *mem.sections[i];
//                tags->bit_mask.resize( (ulong)sec.address_range.size / block_size + 1 );
//                tags->range = sec.address_range;
//                section_tags[i] = std::unique_ptr<SectionTags>( tags );
//            }
//        }
//        
//        void set_ticks( ComputerTime &time, uint read_ticks, uint write_ticks );
//        
//        ulong read( ulong address, uint sec_id );
//        ulong write( ulong address, uint sec_id );
//        
//};
