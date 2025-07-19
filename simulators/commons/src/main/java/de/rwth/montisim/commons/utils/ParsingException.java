/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

public class ParsingException extends RuntimeException {
    private static final long serialVersionUID = -8848593352899284594L;
        char[] data;
        int pos;
        String msg;    
        public ParsingException(char[] data, int pos, String msg) {            
            this.data = data;
            this.pos = pos;            
            this.msg = msg;
        }
        public ParsingException(String msg) {            
            this.data = null;
            this.pos = 0;            
            this.msg = msg;
        }

        @Override
        public String getMessage(){
            if (data == null) return msg;
            StringBuilder line = new StringBuilder();
            line.append(msg);
            line.append(". Context:\n");

            //Get context
            int start = Math.max(0, pos-30);
            int end = Math.min(data.length, pos+30);
            int length = end-start;
            int added = 0;
            // Replace special chars with escaped version
            for (int i = 0; i < length; ++i){
                char c = data[start+i];
                if (c == '\n'){
                    added++;
                    line.append("\\n");
                } else if (c == '\t') {
                    added++;
                    line.append("\\t");
                } else if (c == '\r') {
                    added++;
                    line.append("\\r");
                } else if (c == '\f') {
                    added++;
                    line.append("\\f");
                } else if (c == '\b') {
                    added++;
                    line.append("\\b");
                } else line.append(c);
            }
            // Set marker
            line.append('\n');
            for (int i = 0; i < added + (pos-start-1); ++i) line.append(' ');
            line.append('^');
            return line.toString();
        }
}
