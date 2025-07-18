/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

#include "utility/utility.h"
#include "memory.h"
#include "instruction_time.h"
#include "computer.h"
#include <cmath>
#include <deque>

struct NWayAssociativeCache : public MemoryAccessInterface {

    NWayAssociativeCache(uint level, std::string type, uint N, uint cache_size, uint line_size, ulong read_ticks, ulong write_ticks, ComputerTime& time);

    uint cpu_bit = 64;
    uint level;
    std::string type;

    uint N;
    uint N_bits; // N represented in binary format
    uint cache_size;
    uint line_size;
    uint line_size_bits; // line_size in binary format
    uint cache_line_count; // number of cache lines in cache mem
    uint set_count; // number of sets
    uint set_bits;
    uint set_bit_offset; // offset in the given memory address from where the set_bit starts. set_bit is used to identify the set where the data of that memory address was loaded to
    uint tag_bit_offset; // offset in the given memory address from where the tag_bit starts. tag is used to determine whether the given data in a cache line matches with the data save in the memory address
    ulong set_mask; // mask that is &ed with the memory address to calculate the index of the set where the data of that memory address can be loaded to
    ulong offset_mask;
    ulong tag_bits;
    unsigned int long long tag_mask; // mask that is &ed with the memory address to calculate the corresponding tag

    ulong read_accesses;
    ulong write_accesses;
    ulong read_hits;
    ulong write_hits;
    ulong read_misses;
    ulong write_misses;
    int last_access_result; // 0 == miss, 1 == true, -1 == not accesed
    ulong last_access_address;
    ulong last_access_set_id;

    // time model of the computer
    ComputerTime computer_time;
    ulong read_time;
    ulong write_time;

    std::vector<unsigned int long long> tags; // manages tags of each cache line
    std::vector<std::deque<uint>> lru_queue; // manages Least Recently Used algorithm

    ulong read(ulong address);
    ulong write(ulong address);

    void set_underlying_memory(MemoryAccessInterface* under_mem) {
        this->underlying_memory = under_mem;
    }

    // returns a list of cache-line tags which represents a set in N-Way-Associative Cache
    inline unsigned int long long* get_tags_buffer(int set_id) {
        return tags.data() + ( N * line_size * (ulong)set_id );
    }

    inline std::deque<uint>* get_set_lru_queue(int set_id) {
        return lru_queue.data() + set_id;
    }

    // updates the queue for LRU cache replacement algorithm
    // least recently used cache line -> front
    // most rechently used cache line -> back
    inline void update_lru_queue(int set_id, int line_idx) {
        std::deque<uint>* set_lru_queue = get_set_lru_queue(set_id);
        for (int i = 0 ; i < N ; ++i) {
            if ((*set_lru_queue)[i] == line_idx) {
                set_lru_queue->erase(set_lru_queue->begin() + i);
                set_lru_queue->push_back(line_idx);
                return;
            }
        }
    }

    // returns the LRU line
    inline uint get_LRU_line_idx(int set_id) {
        std::deque<uint>* set_lru_queue = get_set_lru_queue(set_id);

        //load data at memory address to this cache line
        return *(set_lru_queue->begin());
    }

    // returns offset in cache line
    inline ulong get_offset(ulong addr) {
        return addr & offset_mask ;
    }

    // returns the set index to which the memory address belogs to
    inline ulong get_set_idx(ulong addr) {
        return (addr >> set_bit_offset) & set_mask;
    }

    // returns the tag index which correponds to the memory address
    inline unsigned int long long get_tag(ulong addr) {
        return (addr >> tag_bit_offset) & tag_mask;
    }

    inline void load_memory_block(ulong addr, unsigned int long long* set, uint line_idx, ulong offset_in_cache_line, bool modify_bit) {
        auto line_offset = line_idx * line_size;
        ulong mem_block_begin = addr - offset_in_cache_line;
        for (int j = 0 ; j < line_size ; ++j) {
            auto mem_tag = get_tag(mem_block_begin + j);
            auto mem_tag_valid = (unsigned int long long)pow(2, tag_bits + 1) + mem_tag;
            set_modify_bit(&mem_tag_valid, modify_bit);
            /*
            if (modify_bit) {
                mem_tag_valid += (unsigned int long long)pow(2, tag_bits);
                set_modify_bit(&mem_tag_valid, modify_bit);
            }
             */
            auto elem_addr = set + line_offset + j;
            *( elem_addr ) = mem_tag_valid; //10000000000 in memory view 00 00 00 00 00 01
                                                  //                   to read AB CD EF GH IJ KL
                                                  //                      -->  LK JI HG FE DC BA
                                                  //                    little endian
        }
    }

    // Invalid for 0
    inline uint bit_count(uint val) {
        if (val == 0) throw_error("invalid bit conversion for 0");

        uint i = 1;
        while ( !(val <= pow(2,i) ) && i < 64 ) ++i;

        return i;
    }

    inline bool is_power_of_two(uint val) {
        int i = 0;
        while( pow(2,i) <= val ) {
            ++i;
        }

        for (int j = 0 ; j <= i ; j++) {
            if ( pow(2,j) == val ) {
                return true;
            }
        }

        return false;
    }

    // compare two tags without considering modify tag
    inline bool compare_tags ( unsigned long long tag1, unsigned long long tag2 ) {
        auto tag1_tmp = tag1 & ~(unsigned long long)pow(2, tag_bits);
        auto tag2_tmp = tag2 & ~(unsigned long long)pow(2, tag_bits);

        return tag1_tmp == tag2_tmp ? true : false;
    }

    inline bool get_modify_bit( unsigned long long tag ) {
        auto modify_tag = tag & (unsigned long long)pow(2, tag_bits);
        return modify_tag / (unsigned long long)pow(2, tag_bits);
    }

    inline void set_modify_bit( unsigned long long* tag, bool modify ) {
        if (modify) {
            *tag |= (unsigned long long)pow(2, tag_bits);
        }
        else {
            *tag &= ~(unsigned long long)pow(2, tag_bits);
        }
    }
};

// write back policy
// https://course.ccs.neu.edu/com3200/parent/NOTES/cache-basics.html
/*
V = 0, read/write access
        Read from RAM into cache block;
        Set Tag according to RAM address; Set V = 1; Set M = 0;
        Go to V=1, M=0, tag matching, either read or write pending

V = 1, M = 0, tag matching, read pending
        Read from portion of cache block to CPU; Done
V = 1, M = 0, tag matching, write pending
        Write from CPU to portion of cache block; Set M = 1; Done
V = 1, M = 0, tag not matching, read/write pending
        Set V = 0, Go to V = 0, read/write pending

V = 1, M = 1, tag matching, read pending
        Read from portion of cache block to CPU; Done
V = 1, M = 1, tag matching, write pending --> cache hit
        Write from CPU to portion of cache block; Done
V = 1, M = 1, tag not matching, read/write pending --> eviction
        Write cache block to RAM;
Set V = 0; Go to V = 0, read/write pending
Write from CPU to portion of cache block; Done
*/