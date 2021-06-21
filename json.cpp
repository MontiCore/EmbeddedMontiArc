/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "json.h"

#include <stdarg.h>
#include <math.h>
#include <cctype>
#include <limits>
#include <sstream>
#include <inttypes.h>

#include "printf.h"
#include "err_out.h"


const char* get_type_name(ValueType type) {
    switch(type) {
        case ValueType::OBJECT: return "Object";
        case ValueType::ARRAY: return "Array";
        case ValueType::STRING: return "String";
        case ValueType::NUMBER: return "Number";
        case ValueType::BOOLEAN: return "Boolean";
        case ValueType::UNKNOWN: return "Unknown";
    }
    return "";
}


void JsonWriter::start_object() {
    separate();
    if (format) offset += TAB;
    buffer.append('{');
    has_elem = false;
}

void JsonWriter::add_offset(){
    for (int32_t i = 0; i < offset; ++i){
        buffer.append(' ');
    }
}

void JsonWriter::end_object() {
    if (format) {
        offset -= TAB;
        if (has_elem){
            buffer.append('\n');
            add_offset();
        }
    }
    buffer.append('}');
    has_elem = true;
}

void JsonWriter::start_array() {
    separate();
    if (format) offset += TAB;
    buffer.append('[');
    has_elem = false;
}

void JsonWriter::end_array() {
    if (format) {
        offset -= TAB;
        if (has_elem){
            buffer.append('\n');
            add_offset();
        }
    }
    buffer.append(']');
    has_elem = true;
}

void JsonWriter::separate() {
    if (has_elem && !has_key) buffer.append(',');
    else has_elem = true;
    
    if (!has_key) {
        if (format) {
            buffer.append('\n');
            add_offset();
        }
    }
    else has_key = false;
}

void JsonWriter::write_key(const char* key) {
    write_value(key);
    has_key = true;
    if (format) buffer.append(": ");
    else buffer.append(':');
}

void JsonWriter::write_value(const char* str) {
    separate();
    buffer.append('"');
    int32_t i = 0;
    while (str[i]){
        char c = str[i];
        switch(c) {
            case '\n': buffer.append("\\n"); break;
            case '\t': buffer.append("\\t"); break;
            case '\r': buffer.append("\\r"); break;
            case '\f': buffer.append("\\f"); break;
            case '\b': buffer.append("\\b"); break;
            case '"': buffer.append("\\\""); break;
            case '/': buffer.append("\\/"); break;
            case '\\': buffer.append("\\\\"); break;
            default: buffer.append(c); break;
        }
        ++i;
    }
    buffer.append('"');
}

void JsonWriter::write_value(int32_t i){
    separate();
    snprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, "%d", i);
    buffer.append(LOCAL_BUFFER);
}
void JsonWriter::write_value(int64_t l){
    separate();
    snprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, "%lld", l);
    buffer.append(LOCAL_BUFFER);
}
void JsonWriter::write_value(double d){
    separate();
    if (isnan(d)) buffer.append("\"NaN\"");
    else if (isinf(d)) {
        if (d < 0) buffer.append("\"-Infinity\"");
        else buffer.append("\"Infinity\"");
    }
    else {
        snprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, "%.*e", 17, d);
        buffer.append(LOCAL_BUFFER);
    }
}
void JsonWriter::write_value(float d){
    separate();
    if (isnan(d)) buffer.append("\"NaN\"");
    else if (isinf(d)) {
        if (d < 0) buffer.append("\"-Infinity\"");
        else buffer.append("\"Infinity\"");
    }
    else {
        snprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, "%.*e", 9, (double)d); //#define FLT_DECIMAL_DIG  9
        buffer.append(LOCAL_BUFFER);
    }
}
void JsonWriter::write_value(bool b){
    separate();
    if(b) {
        buffer.append("true");
    } else {
        buffer.append("false");
    }
}


bool StringRef::equals(const char* str) const {
    for (int32_t i = 0; i < length; i++) {
        char c = str[i];
        if (c == '\0') return false;
        if (start[i] != c) {
            return false;
        }
    }
    return str[length] == '\0';
}

std::string StringRef::get_json_string() const {
    std::stringstream res;
    for (int32_t i = 0; i < length; ++i){
        auto c = start[i];
        if (c == '\\' && i + 1 < length){
            ++i;
            c = start[i];
            switch(c){
                case 'n': res << ('\n'); break;
                case 't': res << ('\t'); break;
                case 'r': res << ('\r'); break;
                case 'f': res <<('\f'); break;
                case 'b': res <<('\b'); break;
                default: res << (c); break;
            }
        } else res << c;
    }
    return res.str();
}







ObjectIterator::ObjectIterator(JsonTraverser &traverser, int32_t iteration_depth) : traverser(traverser), iteration_depth(iteration_depth), last_elem_pos(traverser.pos) {}
ObjectIterator::ObjectIterator(JsonTraverser &traverser) : traverser(traverser), iteration_depth(-1), last_elem_pos(traverser.pos) {} // For "end()" it.

ObjectIterator& ObjectIterator::operator++() {
    if (last_elem_pos == traverser.pos) {
        traverser.skip_value(); // The last value was not traversed
    }
    while(traverser.depth > iteration_depth){
        // Skip elems until back to this iterator's depth
        traverser.skip_value();
    }
    return *this;
}
StringRef ObjectIterator::operator*() {
    // Get key
    if (traverser.current_type != ValueType::STRING) {
        traverser.parsing_exception("Expected a string as key when iterating object entries.");
    }
    auto key = traverser.get_string();
    last_elem_pos = traverser.pos; // Track traversal
    return key;
}
bool ObjectIterator::operator !=( const ObjectIterator& other ) {
    if (iteration_depth >= 0) {
        return traverser.depth >= iteration_depth;
    } else {
        return traverser.depth >= other.iteration_depth;
    }
}

ArrayIterator::ArrayIterator(JsonTraverser &traverser, int32_t iteration_depth) : traverser(traverser), iteration_depth(iteration_depth), last_elem_pos(traverser.pos) {}
ArrayIterator::ArrayIterator(JsonTraverser &traverser) : traverser(traverser), iteration_depth(-1), last_elem_pos(traverser.pos) {} // For "end()" it.

ArrayIterator& ArrayIterator::operator++() {
    if (last_elem_pos == traverser.pos) {
        traverser.skip_value(); // The last value was not traversed
    }
    while(traverser.depth > iteration_depth){
        // Skip elems until back to this iterator's depth
        traverser.skip_value();
    }
    last_elem_pos = traverser.pos; // Track traversal
    return *this;
}
ValueType ArrayIterator::operator*() {
    return traverser.current_type;
}
bool ArrayIterator::operator!=( const ArrayIterator& other ) {
    if (iteration_depth >= 0) {
        return traverser.depth >= iteration_depth;
    } else {
        return traverser.depth >= other.iteration_depth;
    }
}





void JsonTraverser::parsing_exception(const char *msg_format, ...) {
    va_list args;
    va_start(args, msg_format);
    auto written = vsnprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, msg_format, args);
    va_end(args);
    if (written > LOCAL_BUFFER_SIZE) written = LOCAL_BUFFER_SIZE;
    // Print "    at [Line 6] ... some_json ..."
    int32_t pos_in_line = pos - line_start;
    const char *dots_before = "";
    const char *dots_after = "";
    int32_t before_length = pos_in_line;
    auto context_before = line_start;
    if (pos_in_line > 30) {
        dots_before = "...";
        before_length = 30;
        context_before = pos - 30;
    }
    int32_t after_length = 0;
    auto test = pos;
    while (*test != '\0' && *test != '\n') {
        ++after_length;
        ++test;
        if (after_length >= 50) {
            dots_after = "...";
            break;
        }
    }
    snprintf(LOCAL_BUFFER+written, LOCAL_BUFFER_SIZE-written, "\n  at [Line %" PRIi32 "] %s %.*s[>]%.*s %s", line, dots_before, before_length, context_before, after_length, pos, dots_after);
    ERR_OUT_throw_error("JsonParsingException",LOCAL_BUFFER);
}






JsonTraverser::JsonTraverser(const char *data) {
    pos = data;
    c = *pos;
    depth = 0;
    line = 1;
    line_start = data;
    goto_next_value();
}

bool JsonTraverser::is_empty(){
    if (current_type == ValueType::OBJECT){
        return next_non_ws() == '}';
    } else if (current_type == ValueType::ARRAY) {
        return next_non_ws() == ']';
    } else return false;
}

ObjectStream JsonTraverser::stream_object() {
    if (current_type != ValueType::OBJECT) {
        parsing_exception("Tried to stream an Object but got type %s.", get_type_name(current_type));
    }
    next_char(); // Move after '{'
    depth++;
    int32_t iteration_depth = depth;
    goto_next_value();
    return ObjectStream(*this, iteration_depth);
}

ArrayStream JsonTraverser::stream_array() {
    // TODO
    if (current_type != ValueType::ARRAY) {
        parsing_exception("Tried to stream an Array but got type %s.", get_type_name(current_type));
    }
    next_char(); // Move after '['
    depth++;
    int32_t iteration_depth = depth;
    goto_next_value();
    return ArrayStream(*this, iteration_depth);
}

bool JsonTraverser::get_bool() {
    if (current_type != ValueType::BOOLEAN) {
        parsing_exception("Tried to read a Boolean but got type %s.", get_type_name(current_type));
    }
    skip_bool();
    goto_next_value();
    return last_bool;
}


StringRef JsonTraverser::get_string() {
    if (current_type != ValueType::STRING) {
        parsing_exception("Tried to read a String but got type %s.", get_type_name(current_type));
    }
    auto start = pos +1;
    skip_string();
    auto end = pos - 1;
    goto_next_value();
    return StringRef(start, end - start);
}


double JsonTraverser::get_double() {
    if (current_type == ValueType::NUMBER) {
        char *end;
        auto res = strtod(pos, &end);
        goto_char(end);
        goto_next_value();
        return res;
    } else if (current_type == ValueType::STRING) {
        auto s = get_string();
        if (s.equals("NaN"))
            return nan(nullptr);
        if (s.equals("-Infinity"))
            return -std::numeric_limits<double>::infinity();
        if (s.equals("Infinity"))
            return std::numeric_limits<double>::infinity();
    }
    parsing_exception("Tried to read a Double but got type %s.", get_type_name(current_type));
}



int64_t JsonTraverser::get_long() {
    if (current_type != ValueType::NUMBER) {
        parsing_exception("Tried to read a Long but got type %s.", get_type_name(current_type));
    }
    char *end;
    auto res = strtoll(pos, &end, 10);
    goto_char(end);
    goto_next_value();
    return res;
}

void JsonTraverser::expect_valid_integer(int64_t l) {
    if (l > (int64_t) INT32_MAX || l < (int64_t) INT32_MIN)
        parsing_exception("The int64_t '%" PRIi64 "' doesn't fit in an int32_t", l);
}

void JsonTraverser::get_value_type() {
    if (c == '{')
        current_type = ValueType::OBJECT;
    else if (c == '[')
        current_type = ValueType::ARRAY;
    else if (c == '"')
        current_type = ValueType::STRING;
    else if (c == 't' || c == 'T' || c == 'f' || c == 'F')
        current_type = ValueType::BOOLEAN;
    else if ((c >= '0' && c <= '9') || c == '-' || c == '+')
        current_type = ValueType::NUMBER;
    else
        current_type = ValueType::UNKNOWN;
}

/**
 * Assumes the current position is AFTER a value.
 * Then skips WS and separators and goes to the next VALUE (key or value).
 * Evaluates the Next Value Type.
 */
void JsonTraverser::goto_next_value() {
    skip_whitespace();
    if (c == ':') {
        next_char();
        skip_whitespace();
        get_value_type();
        return;
    }
    do {
        if (c == ',') {
            next_char();
            skip_whitespace();
        }
        if (c == ']' || c == '}') {
            depth--;
            next_char();
            skip_whitespace();
        } else
            break;
    } while (true);
    get_value_type();
}

/*
    SKIP FUNCTIONS

    These must all be called at the START of a value
    and exit at the char AFTER the value
*/

/**
 * Skips any value (and nested values).
 * Goes to the start of the next value
 */
void JsonTraverser::skip_value() {
    int32_t d = depth;
    switch (current_type) {
        case ValueType::ARRAY:
        case ValueType::OBJECT:
            next_char(); // Skip '{' or '['
            depth++;
            goto_next_value();
            while(depth>d && c != '\0'){
                skip_value();
            }
            return;
        case ValueType::BOOLEAN: skip_bool(); break;
        case ValueType::NUMBER: skip_double(); break;
        case ValueType::STRING: skip_string(); break;
        default:
            return;
    }
    goto_next_value();
}

void JsonTraverser::skip_double() {
    while ((c >= '0' && c <= '9') || c == '-' || c == '.' || c == 'E' || c == 'e')
        next_char();
    current_type = ValueType::UNKNOWN;
}
void JsonTraverser::skip_long() {
    if (c == '-')
        next_char();
    if (c < '0' || c > '9')
        parsing_exception("Expected digit in long (got '%c').", c);
    do {
        next_char();
    } while (c >= '0' && c <= '9');
    if (!is_next_whitespace() && c != ',' && c != ']' && c != '}')
        parsing_exception("Unexpected character in Long: '%c'.", c);
    current_type = ValueType::UNKNOWN;
}

void JsonTraverser::skip_string() {
    next_char();
    bool escape = false;
    while (c != '\0') {
        if (escape) {
            escape = false;
        } else if (c == '\\') {
            escape = true;
        } else if (c == '"') {
            break;
        }
        next_char();
    }
    if (c == '\0')
        parsing_exception("Missing closing string delimiter");
    next_char();
    current_type = ValueType::UNKNOWN;
}

void JsonTraverser::skip_bool() {
    if (c == 't' || c == 'T') {
        last_bool = true;
        if (!is_true())
            parsing_exception("Unexpected value for boolean");
    } else {
        last_bool = false;
        if (!is_false())
            parsing_exception("Unexpected value for boolean");
    }
    current_type = ValueType::UNKNOWN;
}


void JsonTraverser::skip_whitespace() {
    while (is_next_whitespace()) {
        next_char();
    }
}

bool JsonTraverser::is_true() {
    next_char();
    if (c != 'r' && c != 'R')
        return false;
    next_char();
    if (c != 'u' && c != 'U')
        return false;
    next_char();
    if (c != 'e' && c != 'E')
        return false;
    next_char();
    return true;
}

bool JsonTraverser::is_false() {
    next_char();
    if (c != 'a' && c != 'A')
        return false;
    next_char();
    if (c != 'l' && c != 'L')
        return false;
    next_char();
    if (c != 's' && c != 'S')
        return false;
    next_char();
    if (c != 'e' && c != 'E')
        return false;
    next_char();
    return true;
}

// PEEKS after the current whitespaces
char JsonTraverser::next_non_ws(){
    auto p = pos;
    char nc;
    do {
        p++;
        nc = *p;
    } while(nc == ' ' || nc == '\n' || nc == '\r' || nc == '\t');
    return nc;
}