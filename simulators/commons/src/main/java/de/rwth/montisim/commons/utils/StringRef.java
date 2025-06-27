/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

public class StringRef {
    public char[] data;
    public int offset;
    public int length;

    public StringRef(char[] data) {
        this.data = data;
        this.offset = 0;
        this.length = 0;
    }
    public StringRef() {
        this.data = null;
        this.offset = 0;
        this.length = 0;
    }

    public StringRef(char[] data, int offset, int length) {
        this.data = data;
        this.offset = offset;
        this.length = length;
    }

    public void init(char[] data){
        this.data = data;
        setEmpty();
    }

    public void setEmpty(){
        this.offset = 0;
        this.length = 0;
    }

    public void set(int offset, int length){
        this.offset = offset;
        this.length = length;
    }

    public boolean equalsIgnoreCase(String str){
        if (str.length() != length) return false;
        for (int i = 0; i < length; i++) {
            char c1 = data[i+offset];
            char c2 = str.charAt(i);
            if (c1 == c2) {
                continue;
            }
            char u1 = Character.toUpperCase(c1);
            char u2 = Character.toUpperCase(c2);
            if (u1 == u2) {
                continue;
            }
            if (Character.toLowerCase(u1) == Character.toLowerCase(u2)) {
                continue;
            }
            return false;
        }
        return true;
    }

    public boolean equals(String str){
        if (str.length() != length) return false;
        for (int i = 0; i < length; i++) {
            if (data[i+offset] != str.charAt(i)) {
                return false;
            }
        }
        return true;
    }

    public int asInt() {
        return Integer.parseInt(getRawString());
        
        // int result = 0;
        // boolean negative = false;
        // int i = 0;
        // int limit = -Integer.MAX_VALUE;
        // int multmin;
        // int digit;

        // if (length <= 0) throw new NumberFormatException("");

        // char firstChar = data[offset];
        // if (firstChar < '0') { // Possible leading "+" or "-"
        //     if (firstChar == '-') {
        //         negative = true;
        //         limit = Integer.MIN_VALUE;
        //     } else if (firstChar != '+')
        //         throw new NumberFormatException(getString());

        //     if (length == 1) // Cannot have lone "+" or "-"
        //         throw new NumberFormatException(getString());
        //     i++;
        // }
        // multmin = limit / 10;
        // while (i < length) {
        //     // Accumulating negatively avoids surprises near MAX_VALUE
        //     digit = Character.digit(data[offset+(i++)],10);
        //     if (digit < 0) {
        //         throw new NumberFormatException(getString());
        //     }
        //     if (result < multmin) {
        //         throw new NumberFormatException(getString());
        //     }
        //     result *= 10;
        //     if (result < limit + digit) {
        //         throw new NumberFormatException(getString());
        //     }
        //     result -= digit;
        // }
        // return negative ? result : -result;
    }

    public long asLong() {
        return Long.parseLong(getRawString());
    }

    public double asDouble() {
        return Double.parseDouble(getRawString());
    }

    public String getRawString() {
        return new String(data, offset, length);
    }

    public String getJsonString() {
        StringBuilder res = new StringBuilder();
        for (int i = 0; i< length; ++i){
            char c = data[offset+i];
            if (c == '\\' && i + 1 < length){
                ++i;
                c = data[offset+i];
                switch(c){
                    case 'n': res.append('\n'); break;
                    case 't': res.append('\t'); break;
                    case 'r': res.append('\r'); break;
                    case 'f': res.append('\f'); break;
                    case 'b': res.append('\b'); break;
                    default: res.append(c); break;
                }
            } else res.append(c);
        }
        return res.toString();
    }
}
