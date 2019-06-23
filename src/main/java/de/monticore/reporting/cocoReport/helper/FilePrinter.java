package de.monticore.reporting.cocoReport.helper;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class FilePrinter {
    private String filePath;
    private int indention = 0;
    private boolean firstPrint = true;

    public FilePrinter(String filePath) {
        this.filePath = filePath;
    }

    public void indent() {
        this.indention++;
    }

    public void unindent() {
        this.indention = this.indention > 0 ? this.indention - 1 : 0;
    }

    public void println(String content) {
        String indentionString = firstPrint ? getIndentionString() : "";
        doPrint(indentionString + content + "\n");
        firstPrint = true;
    }

    public void print(String content) {
        String indentionString = firstPrint ? getIndentionString() : "";
        firstPrint = false;
        doPrint(indentionString + content);
    }

    private String getIndentionString() {
        String indentionString = "";
        for (int i = 0; i < this.indention; i++)
            indentionString += "\t";
        return indentionString;
    }

    private void doPrint(String content) {
        try {
            FileWriter fstream = new FileWriter(filePath, true);
            BufferedWriter out = new BufferedWriter(fstream);
            out.write(content);
            out.close();
        } catch (IOException e) {
            System.err.println("Error while writing to file: " +
                    e.getMessage());
        }
    }
}
