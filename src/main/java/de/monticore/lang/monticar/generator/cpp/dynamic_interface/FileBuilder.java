package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

public class FileBuilder {
    String indentUnit = "    ";
    StringBuilder str = new StringBuilder();

    /**
     * Default indent: 4 spaces
     */
    public FileBuilder() { }
    public FileBuilder(String indent) {
        this.indentUnit = indent;
    }

    /**
     * Resets the internal string builder
     */
    public void init() {
        str.setLength(0);
    }
    
    /**
     * Adds a formatted line to the file (with given indent depth)
     */
    public void a(int indent, String text, Object... data) {
        for (int i = 0; i < indent; ++i) str.append(indentUnit);
        str.append(String.format(text, data));
        str.append('\n');
    }

    /**
     * Adds a formatted line to the file (no indent)
     */
    public void a(String text, Object... data) {
        str.append(String.format(text, data));
        str.append('\n');
    }
    
    /**
     * Returns the resulting content
     */
    public String getContent() {
        return str.toString();
    }
}
