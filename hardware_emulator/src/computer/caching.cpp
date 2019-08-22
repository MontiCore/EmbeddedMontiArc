/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#include "caching.h"




void FifoCache::set_ticks( ComputerTime &time, uint read_ticks, uint write_ticks ) {
    read_time = read_ticks * time.cpu_tick_time_pico;
    write_time = write_ticks * time.cpu_tick_time_pico;
}

ulong FifoCache::read( ulong address, uint sec_id ) {
    auto &tags = *section_tags[sec_id];
    auto local_index = tags.range.get_local_index( address ) / block_size;
    auto tag = tags.bit_mask[local_index];
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
    auto tag = tags.bit_mask[local_index];
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