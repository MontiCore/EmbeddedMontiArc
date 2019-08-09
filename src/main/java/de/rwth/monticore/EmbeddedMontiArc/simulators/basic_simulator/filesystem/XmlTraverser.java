/**
 *
 * ******************************************************************************
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
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
//import java.util.Stack;

public class XmlTraverser {
    //TODO create iterator objects for tags and attributes
    //TODO create Exception object to store position of error in file
    //TODO check EOF checking

    char[] data;
    int pos;
    boolean inside_tag;
    //boolean closing_tag; //Closing tags should be escaped as soon as encountered
    //Stack<String> tag_stack; //Currently no opening/closing validation
    int depth; //Depth of the current character (depth increases as soon as new tag opens ('<'...))
    int traversed_depth; //The depth of the tag currently being inspected

    int last_string_start;
    int last_string_size;

    char current_char;

    public void from_file(File f) {
        data = null;
        if (f.exists()) {
            try {
                FileReader fr;
                fr = new FileReader(f);
                BufferedReader br = new BufferedReader(fr);

                int size = (int) f.length();
                data = new char[size];

                br.read(data);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        reset_traverser();
    }

    public void reset_traverser(){
        pos = 0;
        inside_tag = false;
        //closing_tag = false;
        //tag_stack = new Stack<String>();
        depth = 0;
        traversed_depth = 0;
        current_char = data[0];
    }

    

    private void next_char(){
        pos++;
        if (pos >= data.length) current_char = '\0';
        else current_char = data[pos];
    }

    private void read_string() throws Exception {
        //Validate starting at '"'
        if (!is_next_string_delimiter()){
            last_string_size = 0;
            throw new Exception("Started 'read_string()' not at string delimiter ('\"')");
        }
        boolean escape = false;
        last_string_start = pos + 1;
        do {
            next_char();
            if (!is_next_valid()) throw new Exception("Reached XML EOF while in string");
            if (escape){
                escape = false;
            } else {
                if (is_next_string_delimiter()) break;
                else if (current_char == '\\') escape = true;
            }
        } while(true);
        last_string_size = pos - last_string_start;
        next_char();
        skip_whitespace();
    }


    public String read_keyword() throws Exception {
        //Pos at first char of word
        int word_start = pos;
        while (is_next_valid() && !is_next_whitespace() && !is_next_any_delimiter()){
            next_char();
        }
        int size = pos - word_start;
        skip_whitespace();
        return size == 0 ? "" : new String(data, word_start, size);
    }

    

    //! Returns the name of the next attribute inside the last explored tag or the empty string if none.
    public String next_attribute() throws Exception {
        //Must be inside opening tag
        if (!inside_tag) return ""; //Already escaped tag
        if (is_next_endoftag() || is_next_slash() || !is_next_valid()) return ""; //No attrib left
        String key = read_keyword();
        if (!is_next_equal()) {
            throw new Exception("Expected '=' between attribute key and value.");
        }
        next_char();
        skip_whitespace();
        read_string(); //Stores reference to attrib value for get_attribute_value()
        return key;
    }

    //! Returns value string of the last attribute traversed using next_attribute()
    //! Note: Does not remove the escaping character (ex: \" will stay as is)
    public String get_attribute_value(){
        if (last_string_size == 0) return "";
        return new String(data, last_string_start, last_string_size);
    }

    //! Returns the name of the next tag on the current traversal depth or the empty string if none.
    public String next_tag() throws Exception {
        //NOTE: this method must make sure the 'inside_tag' flag and the depth value are correct
        
        //If inside_tag is true:    Position is after tag name ('/' or '>' or attrib)
        //Else:                     Position is in CONTENT (but not whitespace)
        if (!is_next_valid()) return "";
        
        if (inside_tag){
            internal_exit_tag();
        }
        while (depth > traversed_depth){
            //Skip tags to attain depth
            if (process_tag()){
                skip_keyword();
                internal_exit_tag();
            }
        }
        if (depth < traversed_depth) return ""; //No more tags on the traversal depth

        while (!process_tag()){
            if (!is_next_valid() || depth < traversed_depth) return "";
        }
        return read_keyword();
    }

    //! From CONTENT: goes to next tag type element and checks it
    //! Returns true if it entered a tag (oneliner or opening tag)
    //! Returns false if it skipped the element (and position is set after the element)
    private boolean process_tag() throws Exception {
        to_new_tag();
        if (!is_next_valid()) return false;
        else if (is_next_special()) skip_special();
        else if (is_next_comment()) skip_comment();
        else if (is_next_slash()){
            //Closing tag
            next_char();
            skip_keyword();
            if (!is_next_endoftag()) throw new Exception("Expected '>' at end of closing tag.");
            next_char();
            depth--;
        } else if (is_next_any_delimiter()) throw new Exception("Expected name, '/', '!--' or '?' after '<'.");
        else {
            //New tag
            depth++;
            inside_tag = true;
            return true;
        }
        return false;
    }

    private void internal_exit_tag() throws Exception {
        while (is_next_valid() && !is_next_slash() && !is_next_endoftag()){
            if (is_next_any_delimiter())
                throw new Exception("Expected attrib or '>' or '/>' in tag.");
            skip_attrib();
        }
        if (!is_next_valid()){
            throw new Exception("Reached End-of-File while inside tag.");
        }
        if (is_next_slash()){
            depth--;
            next_char();
        }
        if (!is_next_endoftag()) throw new Exception("Expected '>' at end of tag.");
        next_char();
        inside_tag = false;
    }

    

    //! Sets the traversal depth to that of the current tag
    //! This will make next_tag() return tags of this depth only
    //! Works even if the tag is a one-liner
    //! Returns true if there is content in the tag
    //! Always call exit_tag() to cancel a call to enter_tag()
    public boolean enter_tag() throws Exception {
        //NOTE: this method must make sure the 'inside_tag' flag and the depth value are correct
        traversed_depth++;
        if (inside_tag){
            internal_exit_tag();
        }
        return traversed_depth == depth;
    }

    //! Decreases traversal depth by one
    public void exit_tag(){
        traversed_depth--;
    }


    private void to_new_tag(){
        //Only call in CONTENT
        //Go after next "<" or EOF
        while (is_next_valid() && !is_next_tag()){
            next_char();
        }
        if (is_next_valid()) next_char();
    }

    /*
        SKIP methods move the current position to after a given element
    */

    private void skip_whitespace(){
        while (is_next_whitespace()){
            next_char();
        }
    }

    
    public void skip_keyword() throws Exception {
        while (is_next_valid() && !is_next_whitespace() && !is_next_any_delimiter()){
            next_char();
        }
        skip_whitespace();
    }

    private void skip_attrib() throws Exception {
        skip_keyword();
        if (!is_next_equal()) {
            throw new Exception("Expected '=' between attribute key and value.");
        }
        next_char();
        skip_whitespace();
        read_string();
    }

    private void skip_comment() throws Exception {
        //Searches for "-->"
        //Current is at '!' in "<!--"
        next_char();
        next_char();
        next_char();
        boolean need_dash = false;
        boolean need_delimiter = false;
        do {
            if (is_next_dash()){
                if (need_dash){
                    need_delimiter = true;
                }
                need_dash = true;
            } else if (is_next_endoftag()){
                if (need_delimiter){
                    next_char();
                    return;
                }
                need_dash = false;
                need_delimiter = false;
            } else {
                need_dash = false;
                need_delimiter = false;
            }
            next_char();
        } while (is_next_valid());
        throw new Exception("Reached End-of-File while in comment");
    }

    

    private void skip_special() throws Exception {
        //Searches for "?>"
        //Current pos is at '?' in "<?"
        next_char();
        skip_keyword();
        while (is_next_valid() && !is_next_special()){
            if (is_next_any_delimiter())
                throw new Exception("Expected attrib or '?>' in special tag");
            skip_attrib();
        }
        next_char();
        if (!is_next_valid())
            throw new Exception("Reached End-of-File while in '<? ?>' tag");
        if (!is_next_endoftag())
            throw new Exception("Expected '>' after '?' in special tag");
        next_char();
    }


    /*
        IS methods: check the type of the current character
    */


    private boolean is_next_whitespace(){
        return current_char == ' ' || current_char == '\n' || current_char == '\r' || current_char == '\t';
    }
    public boolean is_next_string_delimiter(){
        return current_char == '"';
    }
    public boolean is_next_valid(){
        return current_char != '\0';
    }
    public boolean is_next_tag(){
        return current_char == '<';
    }
    public boolean is_next_endoftag(){
        return current_char == '>';
    }
    public boolean is_next_slash(){
        return current_char == '/';
    }
    public boolean is_next_dash(){
        return current_char == '-';
    }
    public boolean is_next_special(){
        return current_char == '?';
    }
    public boolean is_next_equal(){
        return current_char == '=';
    }
    public boolean is_next_comment(){
        return current_char == '!';
    }

    public boolean is_next_any_delimiter(){
        return current_char == '=' ||
            current_char == '<' ||
            current_char == '>' ||
            current_char == '/' ||
            current_char == '\\' ||
            current_char == '"' ||
            current_char == '?' ||
            current_char == '!';
    }
}