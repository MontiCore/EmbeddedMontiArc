/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import java.io.*;
import java.util.Iterator;

/**
 * A Lightweight XML Traverser.
 * The goal is to process the content stream only once.
 * 
 * The functions to traverse are:
 * - nextTag()
 * - nextAttribute() -> returns attribute ref
 * - enterTag()
 * - exitTag()
 * 
 * The the current tag name or current attribute key/value can be read and processed.
 */
public class XmlTraverser {
    //TODO check EOF checking

    char[] data;
    int pos;
    boolean inside_tag;
    //boolean closing_tag; //Closing tags should be escaped as soon as encountered
    //Stack<String> tag_stack; //Currently no opening/closing validation
    int depth; //Depth of the current character (depth increases as soon as new tag opens ('<'...))
    int traversed_depth; //The depth of the tag currently being inspected

    char c;

    final StringRef currentTag = new StringRef();
    final StringRef currentKeyword = new StringRef();
    final StringRef currentValue = new StringRef();

    final TagIterable tagIterator = new TagIterable();
    final AttribIterable attribIterator = new AttribIterable();

    public XmlTraverser fromFile(File f) throws FileNotFoundException, IOException{
        data = null;
        FileReader fr;
        fr = new FileReader(f);
        BufferedReader br = new BufferedReader(fr);

        int size = (int) f.length();
        data = new char[size];

        br.read(data);
        br.close();
        resetTraverser();
        return this;
    }

    public void resetTraverser(){
        pos = 0;
        inside_tag = false;
        //closing_tag = false;
        //tag_stack = new Stack<String>();
        depth = 0;
        traversed_depth = 0;
        c = data[0];
        currentKeyword.init(data);
        currentTag.init(data);
        currentValue.init(data);
    }

    public TagIterable tags() {
        return tagIterator;
    }

    public AttribIterable attribs() {
        return attribIterator;
    }

    /** 
     * Returns the name of the next attribute inside the last explored tag or null if none. 
     */
    private StringRef nextAttributeKey() {
        //Must be inside opening tag
        if (!inside_tag) return null; //Already escaped tag
        if (c == '>' || c == '/' || c == '\0') return null; //No attrib left
        read_keyword(currentKeyword);
        if (c != '=') {
            throw new ParsingException(data, pos, "Expected '=' between attribute key and value.");
        }
        next_char();
        skip_whitespace();
        readAttribValue(); //Stores reference to attrib value for get_attribute_value()
        return currentKeyword;
    }

    /** 
     * Returns the value string of the last attribute traversed using next_attribute()
     * Note: Does not remove the escaping character (ex: \" will stay as is)
     */
    public StringRef getAttributeValue() {
        return currentValue;
    }


    

    /** Returns the name of the next tag on the current traversal depth or null if none. */
    private StringRef nextTag() {
        //NOTE: this method must make sure the 'inside_tag' flag and the depth value are correct
        
        //If inside_tag is true:    Position is after tag name ('/' or '>' or attrib)
        //Else:                     Position is in CONTENT (but not whitespace)
        if (c == '\0') return null;
        
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
        if (depth < traversed_depth) return null; //No more tags on the traversal depth

        while (!process_tag()){
            if (c == '\0' || depth < traversed_depth) return null;
        }
        read_keyword(currentTag);
        return currentTag;
    }

    
    
    /**
     * Sets the traversal depth to that of the current tag.
     * This will make next_tag() return tags of this depth only.
     * Works even if the tag is a one-liner.
     * Always call exit_tag() to cancel a call to enter_tag().
     * @return true if there is content in the tag.
     */
    public boolean enterTag() {
        //NOTE: this method must make sure the 'inside_tag' flag and the depth value are correct
        traversed_depth++;
        if (inside_tag){
            internal_exit_tag();
        }
        return traversed_depth == depth;
    }

    /** Exits the previously entered tag. */
    public void exitTag(){
        // Decreases traversal depth by one
        traversed_depth--;
    }

    private void next_char(){
        pos++;
        if (pos >= data.length) c = '\0';
        else c = data[pos];
    }

    private void readAttribValue() {
        //Validate starting at '"'
        if (c != '"'){
            currentValue.setEmpty();
            throw new ParsingException(data, pos, "Started 'read_string()' not at string delimiter ('\"')");
        }
        boolean escape = false;
        int last_string_start = pos + 1;
        do {
            next_char();
            if (c == '\0') throw new ParsingException(data, pos, "Reached XML EOF while in string");
            if (escape){
                escape = false;
            } else {
                if (c == '"') break;
                else if (c == '\\') escape = true;
            }
        } while(true);
        currentValue.set(last_string_start, pos - last_string_start);
        next_char();
        skip_whitespace();
    }


    private void read_keyword(StringRef target) {
        //Pos at first char of word
        int word_start = pos;
        while (c != '\0' && !is_next_whitespace() && !is_next_any_delimiter()){
            next_char();
        }
        int size = pos - word_start;
        skip_whitespace();
        if (size == 0) {
            target.setEmpty();
        } else {
            target.set(word_start, size);
        }
    }

    

    //! From CONTENT: goes to next tag type element and checks it
    //! Returns true if it entered a tag (oneliner or opening tag)
    //! Returns false if it skipped the element (and position is set after the element)
    private boolean process_tag() {
        to_new_tag();
        if (c == '\0') return false;
        else if (c == '?') skip_special();
        else if (c == '!') skip_comment();
        else if (c == '/'){
            //Closing tag
            next_char();
            skip_keyword();
            if (c != '>') throw new ParsingException(data,pos, "Expected '>' at end of closing tag.");
            next_char();
            depth--;
        } else if (is_next_any_delimiter()) throw new ParsingException(data,pos, "Expected name, '/', '!--' or '?' after '<'.");
        else {
            //New tag
            depth++;
            inside_tag = true;
            return true;
        }
        return false;
    }

    private void internal_exit_tag() {
        while (c != '\0' && c != '/' && c != '>'){
            if (is_next_any_delimiter())
                throw new ParsingException(data,pos, "Expected attrib or '>' or '/>' in tag.");
            skip_attrib();
        }
        if (c == '\0'){
            throw new ParsingException(data,pos, "Reached End-of-File while inside tag.");
        }
        if (c == '/'){
            depth--;
            next_char();
        }
        if (c != '>') throw new ParsingException(data,pos, "Expected '>' at end of tag.");
        next_char();
        inside_tag = false;
    }

    



    private void to_new_tag(){
        //Only call in CONTENT
        //Go after next "<" or EOF
        while (c != '\0' && c != '<'){
            next_char();
        }
        if (c != '\0') next_char();
    }

    /*
        SKIP methods move the current position to after a given element
    */

    private void skip_whitespace(){
        while (is_next_whitespace()){
            next_char();
        }
    }

    
    private void skip_keyword() {
        while (c != '\0' && !is_next_whitespace() && !is_next_any_delimiter()){
            next_char();
        }
        skip_whitespace();
    }

    private void skip_attrib() {
        skip_keyword();
        if (c != '=') {
            throw new ParsingException(data, pos, "Expected '=' between attribute key and value.");
        }
        next_char();
        skip_whitespace();
        readAttribValue();
    }

    private void skip_comment() {
        //Searches for "-->"
        //Current is at '!' in "<!--"
        next_char();
        next_char();
        next_char();
        boolean need_dash = false;
        boolean need_delimiter = false;
        do {
            if (c == '-'){
                if (need_dash){
                    need_delimiter = true;
                }
                need_dash = true;
            } else if (c == '>'){
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
        } while (c != '\0');
        throw new ParsingException(data,pos, "Reached End-of-File while in comment");
    }

    

    private void skip_special() {
        //Searches for "?>"
        //Current pos is at '?' in "<?"
        next_char();
        skip_keyword();
        while (c != '\0' && c != '?'){
            if (is_next_any_delimiter())
                throw new ParsingException(data, pos, "Expected attrib or '?>' in special tag");
            skip_attrib();
        }
        next_char();
        if (c == '\0')
            throw new ParsingException(data, pos, "Reached End-of-File while in '<? ?>' tag");
        if (c != '>')
            throw new ParsingException(data, pos, "Expected '>' after '?' in special tag");
        next_char();
    }


    /*
        IS methods: check the type of the current character
    */


    private boolean is_next_whitespace(){
        return c == ' ' || c == '\n' || c == '\r' || c == '\t';
    }
    private boolean is_next_any_delimiter(){
        return c == '=' ||
            c == '<' ||
            c == '>' ||
            c == '/' ||
            c == '\\' ||
            c == '"' ||
            c == '?' ||
            c == '!';
    }

    public class TagIterable implements Iterable<StringRef> {
        public TagIterator it = new TagIterator();
        @Override
        public Iterator<StringRef> iterator() {
            return it;
        }
    }

    public class TagIterator implements Iterator<StringRef> {
        StringRef currentTag = null;
        @Override
        public boolean hasNext() {
            try {
                currentTag = nextTag();
            } catch (ParsingException e) {
                throw new IllegalArgumentException(e.getMessage());
            }
            return currentTag != null;
        }

        @Override
        public StringRef next() {
            return currentTag;
        }
    }

    public class AttribIterable implements Iterable<StringRef> {
        public AttribIterator it = new AttribIterator();
        @Override
        public Iterator<StringRef> iterator() {
            return it;
        }
    }

    public class AttribIterator implements Iterator<StringRef> {
        StringRef currentAttrib = null;
        @Override
        public boolean hasNext() {
            try {
                currentAttrib = nextAttributeKey();
            } catch (ParsingException e) {
                throw new IllegalArgumentException(e.getMessage());
            }
            return currentAttrib != null;
        }

        @Override
        public StringRef next() {
            return currentAttrib;
        }
    }
}