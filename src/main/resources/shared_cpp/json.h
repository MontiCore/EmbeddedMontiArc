/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

#include <string>
//#include <exception>
#include <stdint.h>
#include "buffer.h"


struct JsonWriter {

    static constexpr int32_t TAB = 2;

    DynamicBuffer &buffer;
    
    int32_t offset = 0;
    bool has_elem = false;
    bool has_key = true;
    bool format = true;
    

    JsonWriter(DynamicBuffer &buffer) : buffer(buffer) {}

    const char* get_string() {
        return buffer.as_terminated_string();
    }

    void start_object();
    void end_object();

    void start_array();
    void end_array();

    void write_key(const char* key);
    void write_value(const char* str);
    void write_str(const std::string &str) {
        write_value(str.c_str());
    }

    void write_value(int32_t i);
    void write_value(int64_t l);
    void write_value(double d);
    void write_value(float d);
    void write_value(bool b);

    void write(const char* key, const char* str){
        write_key(key);
        write_value(str);
    }

    void write(const char* key, int32_t i){
        write_key(key);
        write_value(i);
    }

    void write(const char* key, int64_t l){
        write_key(key);
        write_value(l);
    }
    void write(const char* key, float f){
        write_key(key);
        write_value(f);
    }
    void write(const char* key, double d){
        write_key(key);
        write_value(d);
    }
    void write(const char* key, bool b){
        write_key(key);
        write_value(b);
    }

    
private:
    void separate();
    void add_offset();
};










struct StringRef {
    const char* start = nullptr;
    int32_t length = 0;

    StringRef(const char* start, int32_t length) : start(start), length(length) {}

    // bool equals_ignore_case(String str){
    //     if (str.length() != length) return false;
    //     for (int32_t i = 0; i < length; i++) {
    //         char c1 = data[i+offset];
    //         char c2 = str.charAt(i);
    //         if (c1 == c2) {
    //             continue;
    //         }
    //         char u1 = Character.toUpperCase(c1);
    //         char u2 = Character.toUpperCase(c2);
    //         if (u1 == u2) {
    //             continue;
    //         }
    //         if (Character.toLowerCase(u1) == Character.toLowerCase(u2)) {
    //             continue;
    //         }
    //         return false;
    //     }
    //     return true;
    // }

    bool equals(const char* str) const;

    std::string get_json_string() const;

    operator std::string() const {
        return get_json_string();
    }
};

// class ParsingException : public std::exception {
//     const char *msg;
// public:
//     ParsingException(const char *msg) : msg(msg) {}
//     virtual const char* what() const throw()
//     {
//         return msg;
//     }
// };

enum class ValueType {
    OBJECT, ARRAY, STRING, NUMBER, BOOLEAN, UNKNOWN
};

struct JsonTraverser;

struct ObjectIterator {
    JsonTraverser &traverser;
    int32_t iteration_depth;
    const char *last_elem_pos;

    ObjectIterator(JsonTraverser &traverser, int32_t iteration_depth);
    ObjectIterator(JsonTraverser &traverser); // For "end()" it.

    ObjectIterator& operator++();
    StringRef operator*();
    bool operator !=( const ObjectIterator& other );

};

struct ObjectStream {
    JsonTraverser &traverser;
    int32_t iteration_depth;
    ObjectStream(JsonTraverser &traverser, int32_t iteration_depth) : traverser(traverser), iteration_depth(iteration_depth) {}
    ObjectIterator begin(){
        return ObjectIterator(traverser, iteration_depth);
    }
    ObjectIterator end(){
        return ObjectIterator(traverser);
    }
};


struct ArrayIterator {
    JsonTraverser &traverser;
    int32_t iteration_depth;
    const char* last_elem_pos;

    ArrayIterator(JsonTraverser &traverser, int32_t iteration_depth);
    ArrayIterator(JsonTraverser &traverser); // For "end()" it.

    ArrayIterator& operator++();
    ValueType operator*();
    bool operator !=( const ArrayIterator& other );

};

struct ArrayStream {
    JsonTraverser &traverser;
    int32_t iteration_depth;
    ArrayStream(JsonTraverser &traverser, int32_t iteration_depth) : traverser(traverser), iteration_depth(iteration_depth) {}
    ArrayIterator begin(){
        return ArrayIterator(traverser, iteration_depth);
    }
    ArrayIterator end(){
        return ArrayIterator(traverser);
    }
};


struct JsonTraverser {
    friend struct ObjectIterator;
    friend struct ArrayIterator;
    
    JsonTraverser(const char *data);

    ValueType get_type() {
        return current_type;
    }
    bool is_empty();

    ObjectStream stream_object();
    ArrayStream stream_array();
    bool get_bool();
    StringRef get_string();
    double get_double();
    int64_t get_long();

    void expect_valid_integer(int64_t l);

    void parsing_exception(const char *msg_format, ...);

private:

    //const char *data;
    const char* pos;
    int32_t depth;
    ValueType current_type;
    char c;
    bool last_bool;
    int32_t line;
    const char* line_start;


    void get_value_type();

    /**
     * Assumes the current position is AFTER a value.
     * Then skips WS and separators and goes to the next VALUE (key or value).
     * Evaluates the Next Value Type.
     */
    void goto_next_value();

    /*
        SKIP FUNCTIONS

        These must all be called at the START of a value
        and exit at the char AFTER the value
    */

    /**
     * Skips any value (and nested values).
     * Goes to the start of the next value
     */
    void skip_value();
    void skip_double();
    void skip_long();
    void skip_string();
    void skip_bool();
    void skip_whitespace();

    bool is_true();
    bool is_false();
    bool is_next_whitespace() {
        return c == ' ' || c == '\n' || c == '\r' || c == '\t';
    }
    void next_char() {
        if (c == '\0') return;
        if (c != '\n') {
            c = *(++pos);
        } else {
            ++line;
            c = *(++pos);
            line_start = pos;
        }
    }
    // Must make sure it is in range
    void goto_char(const char *target) {
        pos = target;
        c = *pos;
    }
    // PEEKS after the current whitespaces
    char next_non_ws();
};
