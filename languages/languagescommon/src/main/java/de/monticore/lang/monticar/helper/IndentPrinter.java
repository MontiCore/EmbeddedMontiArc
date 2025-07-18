/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.helper;

/**
 * Created by Michael von Wenckstern on 01.06.2016.
 */
public class IndentPrinter {

    protected int indent;
    protected String spacer;
    protected String sp;
    protected int maxlinelength;
    protected boolean optionalBreak;
    protected StringBuilder linebuffer;
    protected StringBuilder writtenbuffer;
    private int optionalBreakPosition;

    public IndentPrinter() {
        this(new StringBuilder());
    }

    public IndentPrinter(StringBuilder writtenbuffer) {
        this.indent = 0;
        this.spacer = "";
        this.sp = "  ";
        this.maxlinelength = -1;
        this.optionalBreak = false;
        this.optionalBreakPosition = -1;
        this.linebuffer = new StringBuilder();
        this.writtenbuffer = writtenbuffer;
    }

    public IndentPrinter(String startContent, int indention) {
        this();
        this.indent(indention);
        this.addLine(startContent);
    }

    public static IndentPrinterGroup groups(String... groups) {
        return new IndentPrinterGroup(groups);
    }

    public String getContent() {
        this.flushBuffer();
        return this.writtenbuffer.toString();
    }

    public int getIndentLength() {
        return this.sp.length();
    }

    public void setIndentLength(int l) {
        this.sp = "";
        int i;
        for (i = 0; i < l; ++i) {
            this.sp = this.sp + " ";
        }
        this.spacer = "";
        for (i = 0; i < this.indent; ++i) {
            this.spacer = this.spacer + this.sp;
        }
    }

    protected void doPrint(String s) {
        for (int pos = s.indexOf("\n"); pos >= 0; pos = s.indexOf("\n")) {
            String substring = s.substring(0, pos);
            if (this.maxlinelength != -1 && pos + this.linebuffer.length() > this.maxlinelength) {
                this.handleOptionalBreak();
            }
            this.linebuffer.append(substring);
            this.flushBuffer();
            this.writtenbuffer.append("\n");
            s = s.substring(pos + 1);
        }
        if (this.maxlinelength != -1 && s.length() + this.linebuffer.length() > this.maxlinelength) {
            this.handleOptionalBreak();
        }
        this.linebuffer.append(s);
    }

    private void handleOptionalBreak() {
        if (this.optionalBreak) {
            if (this.optionalBreakPosition > 0) {
                String sub2 = this.linebuffer.substring(this.optionalBreakPosition);
                this.linebuffer = this.linebuffer.delete(this.optionalBreakPosition, this.linebuffer.length());
                this.flushBuffer();
                this.writtenbuffer.append("\n");
                this.linebuffer.append(sub2);
            }
        } else {
            this.flushBuffer();
            this.writtenbuffer.append("\n");
        }

    }

    public void flushBuffer() {
        this.optionalBreakPosition = 0;
        if (this.linebuffer.length() != 0) {
            this.writtenbuffer.append(this.spacer);
            this.writtenbuffer.append(this.linebuffer);
        }
        this.linebuffer.setLength(0);
    }

    public IndentPrinter indent(int i) {
        if (i > 0) {
            this.indent += i;
            for (int j = 0; j < i; ++j) {
                this.spacer = this.spacer + this.sp;
            }
        } else if (i < 0) {
            while (i < 0 && this.indent > 0) {
                --this.indent;
                this.spacer = this.spacer.substring(this.sp.length());
                ++i;
            }
        }
        return this;
    }

    public IndentPrinter indent() {
        ++this.indent;
        this.spacer = this.spacer + this.sp;
        return this;
    }

    public IndentPrinter unindent() {
        if (this.indent > 0) {
            --this.indent;
            this.spacer = this.spacer.substring(this.sp.length());
        }
        return this;
    }

    public IndentPrinter print(Object o) {
        this.doPrint(o == null ? "null" : o.toString());
        return this;
    }

    public IndentPrinter printWithoutProcessing(Object o) {
        this.linebuffer.append(o.toString());
        return this;
    }

    public IndentPrinter println(Object o) {
        this.print(o);
        this.println();
        return this;
    }

    public IndentPrinter println() {
        this.doPrint("\n");
        return this;
    }

    public IndentPrinter println(int n) {
        for (int i = 0; i < n; ++i) {
            this.doPrint("\n");
        }
        return this;
    }

    public StringBuilder getWrittenbuffer() {
        return this.writtenbuffer;
    }

    public boolean isStartOfLine() {
        return this.linebuffer.length() == 0;
    }

    public IndentPrinter addLine(Object newContent) {
        String nc = newContent.toString().trim();
        int counter = 0;
        for (int i = 0; i < nc.length(); ++i) {
            if (nc.charAt(i) == 123) {
                ++counter;
            } else if (nc.charAt(i) == 125) {
                --counter;
            }
        }
        if (counter < 0) {
            this.indent(counter);
        }
        this.print(newContent);
        this.println();
        if (counter > 0) {
            this.indent(counter);
        }
        return this;
    }

    public int getMaxlinelength() {
        return this.maxlinelength;
    }

    public void setMaxlinelength(int maxlinelength) {
        this.maxlinelength = maxlinelength;
    }

    public boolean isOptionalBreak() {
        return this.optionalBreak;
    }

    public IndentPrinter setOptionalBreak(boolean optionBreak) {
        this.optionalBreak = optionBreak;
        return this;
    }

    public IndentPrinter optionalBreak() {
        this.optionalBreakPosition = this.linebuffer.length();
        return this;
    }

    public IndentPrinter clearBuffer() {
        this.flushBuffer();
        this.writtenbuffer.setLength(0);
        return this;
    }
}
