/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "debug.h"
#include "computer/caching.h"
using namespace std;

std::string zydis_registers[] = {
#include "zydis_registers.table"
};

void ComputerDebug::init( Memory &mem, Registers &regs, CodeDecoder &decoder ) {
    this->memory = &mem;
    this->registers = &regs;
    this->decoder = &decoder;
}

void ComputerDebug::debug_syscall( SysCall &sys_call, ulong id ) {
    if ( !syscalls() )
        return;

    Log::sys.log_tag("%s sys_call: %llu %s!%s", sys_call.supported() ? "Supported" : "Unsupported", id, sys_call.module.c_str(), sys_call.name.c_str());
}

void ComputerDebug::debug_unsupp_syscall( SysCall &sys_call ) {
    if ( !unsupported_syscalls() )
        return;
    Log::sys.log_tag("Unsupported sys_call: %s!%s", sys_call.module.c_str(), sys_call.name.c_str());
}

void ComputerDebug::debug_code( ulong addr, uint size, ulong ticks, ulong time ) {
    if ( !debug || !d_code )
        return;
        
    //Print the register updates now (for the last instruction), since there is no callback for after an instruction execution.
    if ( d_regs )
        registers->print_registers();
    else if ( d_reg_update )
        registers->print_changed_registers();
        
    char annot[256];
    memory->print_annotation(addr, annot, 256);
    if ( decoder->succeeded ) {
        // Print binary instruction code.
        std::string res;
        for ( uint i : urange( size ) )
            res += to_hex( ( uint64_t )( decoder->code[i] ), 2 );
        // Format & print the binary instruction structure to human readable format
        // the resolution of the time parameter is pico second
        // dividing time with 1000 gives nanosecond resolution, but discards details
        // but the discarded details are not ignorable when they are all summed up
        //Log::code.log_tag("%s   %-21s%-40sticks: %llu (time: %lluns)    %s", to_hex(addr).c_str(), res.c_str(), buffer.data(), ticks, (time / 1000), annot);
        Log::code.log_tag("%s   %-21s%-40sticks: %llu (time: %.3fns)   %s", to_hex(addr).c_str(), res.c_str(), decoder->buffer.data(), ticks, ((double)time / 1000), annot); // annot is the annotation for the accessed memory address
    }
    else {
        Log::code.log_tag("ticks: %llu (time: %.3fns)    %s", ticks, ((double)time / 1000), annot);
    }
}

void ComputerDebug::debug_code_noval( ulong addr, uint size ) {
    if ( decoder->succeeded ) {
        Log::code.log("No time for instruction: %s", decoder->buffer.data());
    }
}

void ComputerDebug::debug_instruction_operands() {
    if (decoder->succeeded && d_instruction_operands) {
        // print out operand information

        auto operands = decoder->instruction.operands;

        for (int i = 0 ; i < ZYDIS_MAX_OPERAND_COUNT ; ++i) {
            auto& operand = operands[i];
            if (operand.type == ZYDIS_OPERAND_TYPE_UNUSED) continue;

            //resolve operand type
            basic_string<char> operand_type = ComputerDebug::resolve_operand_type(this->decoder, this, operand.type);
            basic_string<char> element_type = ComputerDebug::resolve_operand_element_type(this->decoder, this, operand.element_type);
            Log::instruction_operands.log_tag("%47s 0%d", "operand", i);
            Log::instruction_operands.log_tag("%48s: %s", "type", operand_type.c_str());
            Log::instruction_operands.log_tag("%48s: %d", "size", operand.size);
            Log::instruction_operands.log_tag("%56s: %s", "element type", element_type.c_str());
            Log::instruction_operands.log_tag("%56s: %d", "element size", operand.element_size);

            if (d_operands_details) debug_operands_details(operand);
        }
    }
}

basic_string<char> ComputerDebug::resolve_operand_type(CodeDecoder* decoder, ComputerDebug* debug, ZydisOperandType type) {
    // debug == nullptr means that this function was called from instruction_time.cpp

    if (!decoder->succeeded) throw_error("trying to resolve operands after unsuccessful decoding");
    //if (type == ZYDIS_OPERAND_TYPE_UNUSED) throw_error("trying to resolve UNUSED operand");
    if (debug) {
        if (!debug->d_instruction_operands) throw_error("debug instruction operands not requested");
    }

    switch (type) {
        case ZYDIS_OPERAND_TYPE_REGISTER:
            if (debug) return std::string("REGISTER");
            else return std::string("r");
        case ZYDIS_OPERAND_TYPE_MEMORY:
            if (debug) return std::string("MEMORY");
            else return std::string("m");
        case ZYDIS_OPERAND_TYPE_POINTER:
            if (debug) return std::string("POINTER");
            else return std::string("p");
        case ZYDIS_OPERAND_TYPE_IMMEDIATE:
            if (debug) return std::string("IMMEDIATE");
            else return std::string("i");
        default:
            if (debug) return std::string("u");
            return std::string("UNUSED   ");
    }
}

basic_string<char> ComputerDebug::resolve_operand_element_type(CodeDecoder* decoder, ComputerDebug* debug, ZydisElementType element_type) {
    // debug == nullptr means that this function was called from instruction_time.cpp

    if (!decoder->succeeded) throw_error("trying to resolve operands after unsuccessful decoding");
    if (element_type == ZYDIS_ELEMENT_TYPE_INVALID) throw_error("trying to resolve INVALID element type operand");
    // when this function is used in debugging
    if (debug) {
        if (!debug->d_instruction_operands) throw_error("debug instruction operands not requested");
    }

    switch (element_type) {
        case ZYDIS_ELEMENT_TYPE_STRUCT:
            return std::string("STRUCT");
        case ZYDIS_ELEMENT_TYPE_UINT:
            return std::string("UINT");
        case ZYDIS_ELEMENT_TYPE_INT:
            return std::string("INT");
        case ZYDIS_ELEMENT_TYPE_FLOAT16:
            return std::string("FLOAT16");
        case ZYDIS_ELEMENT_TYPE_FLOAT32:
            return std::string("FLOAT32");
        case ZYDIS_ELEMENT_TYPE_FLOAT64:
            return std::string("FLOAT64");
        case ZYDIS_ELEMENT_TYPE_FLOAT80:
            return std::string("FLOAT80");
        case ZYDIS_ELEMENT_TYPE_LONGBCD:
            return std::string("LONGBCD");
        case ZYDIS_ELEMENT_TYPE_CC:
            return std::string("CONDITION CODE");
        default:
            return std::string("INVALID");
    }
}

basic_string<char> ComputerDebug::resolve_operand_size(CodeDecoder* decoder, ZydisElementSize size) {
    if (!decoder->succeeded) throw_error("trying to resolve operands after unsuccessful decoding");
    return to_string(size);
}

basic_string<char> ComputerDebug::resolve_element_size(CodeDecoder* decoder, ZydisElementSize size) {
    if (!decoder->succeeded) throw_error("trying to resolve element type of operands of undefined instruction");
    return to_string(size);
}

 void ComputerDebug::debug_operands_details(ZydisDecodedOperand &operand) {

    Log::instruction_operands.log_tag("%51s:", "details");

    if (operand.type == ZYDIS_OPERAND_TYPE_REGISTER){
        Log::instruction_operands.log_tag("%61s: %s", "register name", zydis_registers[operand.reg.value].c_str());
    }
    else if (operand.type == ZYDIS_OPERAND_TYPE_POINTER){
        Log::instruction_operands.log_tag("%63s: %d", "pointer segment", operand.ptr.segment);
        Log::instruction_operands.log_tag("%62s: %d", "pointer offset", operand.ptr.offset);
    }
    else if (operand.type == ZYDIS_OPERAND_TYPE_IMMEDIATE){
        Log::instruction_operands.log_tag("%57s: %d", "is_signed", operand.imm.is_signed);
        Log::instruction_operands.log_tag("%59s: %d", "is_relative", operand.imm.is_relative);
        Log::instruction_operands.log_tag("%55s: %d", "value_s", operand.imm.value.s);
        Log::instruction_operands.log_tag("%55s: %u", "value_u", operand.imm.value.u);
    }
    else if (operand.type == ZYDIS_OPERAND_TYPE_MEMORY) {
        std::string memory_operand_type;
        switch (operand.mem.type) {
            case ZYDIS_MEMOP_TYPE_MEM:
                memory_operand_type = "NORMAL";
                break;
            case ZYDIS_MEMOP_TYPE_AGEN:
                memory_operand_type = "ADDRESS GENERATIOAN";
                break;
            case ZYDIS_MEMOP_TYPE_MIB:
                memory_operand_type = "MIB";
                break;
            default:
                throw_error("INVALID memory operand encountered while resolving instruction operand details");
                break;
        }

        std::string segment_reg = zydis_registers[operand.mem.segment];
        std::string base_reg = zydis_registers[operand.mem.base];
        std::string index_reg = zydis_registers[operand.mem.index];
        std::string scale = to_string(operand.mem.scale);
        std::string has_displacement = to_string(operand.mem.disp.has_displacement);
        std::string displacement = to_string(operand.mem.disp.has_displacement);
        Log::instruction_operands.log_tag("%67s: %s", "memory operand type", memory_operand_type.c_str());
        Log::instruction_operands.log_tag("%59s: %s", "segment_reg", segment_reg.c_str());
        Log::instruction_operands.log_tag("%56s: %s", "base_reg", base_reg.c_str());
        Log::instruction_operands.log_tag("%57s: %s", "index_reg", index_reg.c_str());
        Log::instruction_operands.log_tag("%53s: %s", "scale", scale.c_str());
        Log::instruction_operands.log_tag("%64s: %s", "has_displacement", has_displacement.c_str());
        Log::instruction_operands.log_tag("%60s: %s", "displacement", displacement.c_str());
    }
    else {
        throw_error("Unexpected operand type");
    }
}

void ComputerDebug::debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val ) {
    char addr_info[256];
    memory->print_address_info(addr, addr_info, 256);
    if (type == MemAccess::WRITE) {
        Log::err.log_tag("Invalid memory %s at %s with value %lli of size %lu %s", 
            (type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" : "fetch"),
            to_hex(addr, 16, true).c_str(),
            val,
            size,
            addr_info
        );
    }
    else {
        Log::err.log_tag("Invalid memory %s at %s of size %lu %s",
            (type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" : "fetch"),
            to_hex(addr, 16, true).c_str(),
            size,
            addr_info
        );
    }
}

void ComputerDebug::debug_mem( MemAccess type, ulong addr, uint size, slong val, ulong time ) {
    if ( !debug || !d_mem )
        return;
    //Print memory access type
    const char* tag;
    switch ( type ) {
        case MemAccess::READ: tag = Log::mem_read.tag; break;
        case MemAccess::WRITE: tag = Log::mem_write.tag; break;
        case MemAccess::FETCH: tag = Log::mem_fetch.tag; break;
        default: tag = Log::info.tag; break;
    }
    
    
    char addr_info[256];
    memory->print_address_info(addr, addr_info, 256);
    if (type == MemAccess::WRITE) {
        Log::info.log("%-7s%s   %s                                             time: %llu.%lluns     %s",
            tag, to_hex(addr).c_str(), to_hex(val).c_str(), time / 1000, time % 1000, addr_info);
    }
    else {
        //Print memory content
        uint32_t s = size;
        uchar* data = (uchar*)memory->read_memory(addr, s);
        std::string res;
        for (uint64_t i : ulrange(s)) {
            uint8_t v = *((uint8_t*) & (data[s - 1 - i]));
            res += to_hex((uint64_t)v, 2);
        }
        Log::info.log("%-7s%s   %-21s                                        time: %llu.%lluns     %s",
            tag, to_hex(addr).c_str(), res.c_str(), time / 1000, time % 1000, addr_info);
    }

}

void ComputerDebug::debug_call( ulong address, const char *name ) {
    if ( call() )
        Log::debug.log("[CALL] %s() at %s", name, to_hex(address).c_str());
}

void ComputerDebug::debug_register_syscall( SysCall const &call, ulong addr, const char *reason ) {
    if ( !debug || !d_syscalls )
        return;
        
    Log::sys.log_tag("Added Syscall for %s: %s!%s  %s", reason, call.module.c_str(), call.name.c_str(), to_hex((ulong)addr, 3).c_str());
             
    if ( undercorate_function_name( call.name, decoder->buffer )
            && call.name.compare( decoder->buffer.data() ) != 0 )
        Log::debug.log("%s", decoder->buffer.data());
}


void ComputerDebug::debug_cache_hit_ratio(MemoryModel *mem_model) {
    //iterate from bigger index
    if (!d_cache_hit_ratio) return;

    uint layers_count = mem_model->get_layer_count();
    for( int i = layers_count - 1 ; i >= 0 ; i--) {
        NWayAssociativeCache* layer = dynamic_cast<NWayAssociativeCache*>(mem_model->get_memory_layer(i));
        uint level = layer->level;
        std::string type = layer->type;
        ulong total_accesses = layer->read_accesses + layer->write_accesses;
        ulong total_hits = layer->read_hits + layer->write_hits;
        ulong total_misses = layer->read_misses + layer->write_misses;
        Log::cache_hit_ratio.log_tag("    L%d%s accesses: %15lu (R: %15lu + W: %15lu)", level, type.c_str(), total_accesses, layer->read_accesses, layer->write_accesses);
        Log::cache_hit_ratio.log_tag("        hits:     %15lu (R: %15lu + W: %15lu)", total_hits, layer->read_hits, layer->write_hits);
        Log::cache_hit_ratio.log_tag("        misses:   %15lu (R: %15lu + W: %15lu)", total_misses, layer->read_misses, layer->write_misses);
    }
}

void ComputerDebug::debug_memory_access(MemoryModel *mem_model) {

    if (!d_mem_access) return;

    uint layers_count = mem_model->get_layer_count();

    for( int i = layers_count - 1 ; i >= 0 ; i--) {
        NWayAssociativeCache* layer = dynamic_cast<NWayAssociativeCache*>(mem_model->get_memory_layer(i));
        if (layer->last_access_result != -1) {
            uint level = layer->level;
            std::string type = layer->type;
            std::string access_result = layer->last_access_result == 1 ? "hit" : "miss" ;
            Log::mem_access.log_tag("%-79s accessed: L%d%s    result: %s", "", level, type.c_str(), access_result.c_str());
            //Log::mem_access.log_tag("addr: %lu, set_id : %lu", layer->last_access_address, layer->last_access_set_id);
            // set the layer to "unaccessed"
            layer->last_access_result = -1;
        }
    }
}