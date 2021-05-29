/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils.json;

import java.io.*;
import java.util.Iterator;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.StringRef;
import de.rwth.montisim.commons.utils.json.Json;

/**
 * @throws ParsingException
 */
public class JsonTraverser {
    private static final int PRE_DEPTH = 20;

    char[] data;
    char c;
    int pos;
    int depth;
    ValueType currentType;
    boolean lastBool;

    final StringRef currentKey = new StringRef();
    final StringRef currentString = new StringRef();

    final ObjectIterable[] objIterators = new ObjectIterable[PRE_DEPTH];
    final ArrayIterable[] arrayIterators = new ArrayIterable[PRE_DEPTH];

    public static enum ValueType {
        OBJECT, ARRAY, STRING, NUMBER, BOOLEAN, UNKNOWN
    }

    public void reset() {
        pos = 0;
        depth = 0;
        c = data.length == 0 ? '\0' : data[0];
        currentKey.init(data);
        currentString.init(data);
        gotoNextValue();
    }

    public JsonTraverser init(File file) throws IOException, ParsingException  {
        data = null;
        if (file.exists()) {
            FileReader fr;
            fr = new FileReader(file);
            BufferedReader br = new BufferedReader(fr);

            int size = (int) file.length();
            data = new char[size];

            br.read(data);
            br.close();
        }
        reset();
        return this;
    }

    public JsonTraverser init(String json) throws IOException {
        data = json.toCharArray();
        reset();
        return this;
    }
    
    public ValueType getType() {
        return currentType;
    }

    private char nextNonWS(){
        int p = pos;
        char nc = '\0';
        do {
            p++;
            nc = p >= data.length ? '\0' : data[p];
        } while(nc == ' ' || nc == '\n' || nc == '\r' || nc == '\t');
        return nc;
    }

    public boolean isEmpty(){
        if (currentType == ValueType.OBJECT){
            return nextNonWS() == '}';
        } else if (currentType == ValueType.ARRAY) {
            return nextNonWS() == ']';
        } else return false;
    }

    public ObjectIterable streamObject() throws ParsingException {
        if (currentType != ValueType.OBJECT) throw new ParsingException(data, pos, "Tried to read OBJECT but got "+currentType);
        nextChar(); // Move after '{'
        depth++;
        int itDepth = depth;
        gotoNextValue();
        if (itDepth >= PRE_DEPTH) return new ObjectIterable(itDepth);
        ObjectIterable it = objIterators[itDepth];
        if (it == null){
            it = new ObjectIterable(itDepth);
            objIterators[itDepth] = it;
        }
        it.it.reset();
        return it;
    }

    public ArrayIterable streamArray() throws ParsingException {
        if (currentType != ValueType.ARRAY) throw new ParsingException(data, pos, "Tried to read ARRAY but got "+currentType);
        nextChar(); // Move after '['
        depth++;
        int itDepth = depth;
        gotoNextValue();
        if (itDepth >= PRE_DEPTH) return new ArrayIterable(itDepth);
        ArrayIterable it = arrayIterators[itDepth];
        if (it == null){
            it = new ArrayIterable(itDepth);
            arrayIterators[itDepth] = it;
        }
        it.it.reset();
        return it;
    }

    public boolean getBoolean() throws ParsingException {
        if (currentType != ValueType.BOOLEAN) throw new ParsingException(data, pos,  "Tried to read BOOLEAN but got "+currentType);
        skipBoolean();
        gotoNextValue();
        return lastBool;
    }


    public StringRef getString() throws ParsingException {
        if (currentType != ValueType.STRING) throw new ParsingException(data, pos, "Tried to read STRING but got "+currentType);
        getString(currentString);
        return currentString;
    }


    public double getDouble() throws ParsingException {
        int p = pos;
        if (currentType == ValueType.NUMBER) {
            return parseDouble();
        } else if (currentType == ValueType.STRING) {
            getString(currentString);
            if (currentString.equals("NaN"))
                return Double.NaN;
            if (currentString.equals("-Infinity"))
                return Double.NEGATIVE_INFINITY;
            if (currentString.equals("Infinity"))
                return Double.POSITIVE_INFINITY;
        }
        throw new ParsingException(data, p, "Tried to read DOUBLE but got "+currentType);
    }

    private double parseDouble() throws ParsingException {
        int start = pos;
        skipDouble();
        int end = pos;
        gotoNextValue();
        return Double.parseDouble(new String(data, start, end - start));
    }


    public long getLong() throws ParsingException {
        if (currentType != ValueType.NUMBER) throw new ParsingException(data, pos, "Tried to read LONG but got "+currentType);
        int start = pos;
        skipLong();
        int end = pos;
        gotoNextValue();
        return Long.parseLong(new String(data, start, end - start));
    }

    public Pair<String, ObjectIterable> getStructureType() throws ParsingException {
        ObjectIterable it = streamObject();
        if (!it.iterator().hasNext()) expected(Json.K_TYPE);
        Entry e = it.iterator().next();
        if (!e.key.getJsonString().equals(Json.K_TYPE)) expected(Json.K_TYPE);
        String type = getString().getJsonString();
        return new Pair<String, ObjectIterable>(type, it);
    }

    public ObjectIterable expectStructureType(String type) throws ParsingException {
        ObjectIterable it = streamObject();
        if (!it.iterator().hasNext()) expected(Json.K_TYPE);
        Entry e = it.iterator().next();
        if (!e.key.getJsonString().equals(Json.K_TYPE)) expected(Json.K_TYPE);
        StringRef t = getString();
        if (!t.equals(type)) expectedStructureType(type, t.getRawString());
        return it;
    }

    public void expectedStructureType(String type, String gotten) throws ParsingException {
        throw new ParsingException("Unexpected structure type: "+gotten+", expected: "+type);
    }

    public void expectValidInteger(long l) throws ParsingException {
        if (l > (long) Integer.MAX_VALUE || l < (long) Integer.MIN_VALUE)
            throw new NumberFormatException("The long doesn't fit in an int");
    }

    public void expected(String entryName) throws ParsingException {
        throw new ParsingException(data, pos, "Expected entry " + entryName);
    }

    public void expected(ValueType type) throws ParsingException {
        throw new ParsingException(data, pos, "Expected value of type "+type+" but got type " + currentType);
    }
    public void unexpected(Entry e) throws ParsingException {
        throw new ParsingException(data, pos, "Unexpected entry: "+e.key.getRawString());
    }

    public void throwContextualParsingException(String msg) {
        throw new ParsingException(data, pos, msg);
    }



    private void getValueType() {
        if (c == '{')
            currentType = ValueType.OBJECT;
        else if (c == '[')
            currentType = ValueType.ARRAY;
        else if (c == '"')
            currentType = ValueType.STRING;
        else if (c == 't' || c == 'T' || c == 'f' || c == 'F')
            currentType = ValueType.BOOLEAN;
        else if (Character.isDigit(c) || c == '-' || c == '+')
            currentType = ValueType.NUMBER;
        else
            currentType = ValueType.UNKNOWN;
    }
    // public long parseLong() {
    // boolean negative = c == '-';
    // if (negative) nextChar();
    // if (!Character.isDigit(c)) throw new ParsingException(data, pos, "Expected
    // digit in long");
    // long res = 0;
    // do {
    // res = res * 10L + (long) ((int) c - (int) '0');
    // nextChar();
    // } while (Character.isDigit(c));
    // if (!isNextWhitespace() && c != ',' && c != ']' && c != '}') throw new
    // ParsingException(data, pos, "Unexpected character in long");
    // return negative ? -res : res;
    // }
    // public double parseDouble() {
    // boolean negative = c == '-';
    // if (negative) nextChar();
    // if (c != '0') {
    // if(!Character.isDigit(c)) throw new ParsingException(data, pos, "Expected a
    // digit in double");
    // }
    // double res = 0;
    // Double.parseDouble("0");
    // return res;
    // }

    private void getString(StringRef target) {
        int start = pos +1;
        skipString();
        int end = pos - 1;
        target.offset = start;
        target.length = end - start;
        gotoNextValue();
    }

    /**
     * Assumes the current position is AFTER a value.
     * Then skips WS and separators and goes to the next VALUE (key or value).
     * Evaluates the Next Value Type.
     */
    private void gotoNextValue() {
        skipWhitespace();
        if (c == ':') {
            nextChar();
            skipWhitespace();
            getValueType();
            return;
        }
        do {
            if (c == ',') {
                nextChar();
                skipWhitespace();
            }
            if (c == ']' || c == '}') {
                depth--;
                nextChar();
                skipWhitespace();
            } else
                break;
        } while (true);
        getValueType();
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
    private void skipValue() {
        int d = depth;
        switch (currentType) {
            case ARRAY:
            case OBJECT:
                nextChar(); // Skip '{' or '['
                depth++;
                gotoNextValue();
                while(depth>d && c != '\0'){
                    skipValue();
                }
                return;
            case BOOLEAN: skipBoolean(); break;
            case NUMBER: skipDouble(); break;
            case STRING: skipString(); break;
            default:
                return;
        }
        gotoNextValue();
    }

    public void skipDouble() {
        while (((int) c >= (int) '0' && (int) c <= (int) '9') || c == '+' || c == '-' || c == '.' || c == 'E' || c == 'e')
            nextChar();
        currentType = ValueType.UNKNOWN;
    }
    public void skipLong() {
        if (c == '-')
            nextChar();
        if ((int) c < (int) '0' || (int) c > (int) '9')
            throw new ParsingException(data, pos, "Expected digit in long");
        do {
            nextChar();
        } while ((int) c >= (int) '0' && (int) c <= (int) '9');
        if (!isNextWhitespace() && c != ',' && c != ']' && c != '}')
            throw new ParsingException(data, pos, "Unexpected character in long");
        currentType = ValueType.UNKNOWN;
    }

    private void skipString() {
        nextChar();
        boolean escape = false;
        while (c != '\0') {
            if (escape) {
                escape = false;
            } else if (c == '\\') {
                escape = true;
            } else if (c == '"') {
                break;
            }
            nextChar();
        }
        if (c == '\0')
            throw new ParsingException(data, pos, "Missing closing string delimiter");
        nextChar();
        currentType = ValueType.UNKNOWN;
    }

    private void skipBoolean() {
        if (c == 't' || c == 'T') {
            lastBool = true;
            if (!isTrue())
                throw new ParsingException(data, pos, "Unexpected value");
        } else {
            lastBool = false;
            if (!isFalse())
                throw new ParsingException(data, pos, "Unexpected value");
        }
        currentType = ValueType.UNKNOWN;
    }

    
    private void skipWhitespace() {
        while (isNextWhitespace()) {
            nextChar();
        }
    }

    private boolean isTrue() {
        nextChar();
        if (c != 'r' && c != 'R')
            return false;
        nextChar();
        if (c != 'u' && c != 'U')
            return false;
        nextChar();
        if (c != 'e' && c != 'E')
            return false;
        nextChar();
        return true;
    }

    private boolean isFalse() {
        nextChar();
        if (c != 'a' && c != 'A')
            return false;
        nextChar();
        if (c != 'l' && c != 'L')
            return false;
        nextChar();
        if (c != 's' && c != 'S')
            return false;
        nextChar();
        if (c != 'e' && c != 'E')
            return false;
        nextChar();
        return true;
    }


    private boolean isNextWhitespace() {
        return c == ' ' || c == '\n' || c == '\r' || c == '\t';
    }

    private void nextChar() {
        pos++;
        if (pos >= data.length)
            c = '\0';
        else
            c = data[pos];
    }

    public static class Entry {
        public StringRef key;
        public ValueType valueType;
    }

    public class ObjectIterable implements Iterable<Entry> {
        final ObjectEntryStream it;
        ObjectIterable(int itDepth){
            it = new ObjectEntryStream(itDepth);
        }
        @Override
        public Iterator<Entry> iterator() {
            return it;
        }
    }

    class ObjectEntryStream implements Iterator<Entry> {
        final Entry entry = new Entry();
        final int itDepth;
        int lastElemPos;

        ObjectEntryStream(int itDepth){
            this.itDepth = itDepth;
            reset();
        }
        void reset() {
            lastElemPos = -1;
        }

        @Override
        public boolean hasNext() {
            while(depth > itDepth){
                // Skip elems until back to this iterator's depth
                skipValue();
            }
            if (lastElemPos == pos) {
                skipValue(); // The last value was not traversed
            }
            return depth >= itDepth;
        }
        @Override
        public Entry next() {
            while(depth > itDepth){
                // Skip elems until back to this iterator's depth
                skipValue();
            }
            // Get key
            if (currentType != ValueType.STRING) throw new ParsingException(data,pos, "Expected a string as key");
            getString(currentKey);
            entry.key = currentKey;
            entry.valueType = currentType;
            lastElemPos = pos; // Track traversal
            return entry;
        }

    }

    public class ArrayIterable implements Iterable<ValueType> {
        final ArrayEntryStream it;
        ArrayIterable(int itDepth){
            it = new ArrayEntryStream(itDepth);
        }
        @Override
        public Iterator<ValueType> iterator() {
            return it;
        }
    }

    class ArrayEntryStream implements Iterator<ValueType> {
        final int itDepth;
        int lastElemPos;

        ArrayEntryStream(int itDepth){
            this.itDepth = itDepth;
            reset();
        }
        void reset() {
            lastElemPos = -1;
        }

        @Override
        public boolean hasNext() {
            if (lastElemPos == pos) {
                skipValue(); // The last value was not traversed
            }
            return depth >= itDepth;
        }
        @Override
        public ValueType next() {
            while(depth > itDepth){
                // Skip elems until back to this iterator's depth
                skipValue();
            }
            // Get value type
            lastElemPos = pos; // Track traversal
            return currentType;
        }

    }
}
