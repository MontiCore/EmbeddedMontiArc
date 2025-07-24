/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.helper;

import de.monticore.reporting.tools.CustomPrinter;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class FilePrinter {
    private int indention = 0;
    private boolean firstPrint = true;
    private BufferedWriter out;
    private int i;

    public FilePrinter(String filePath) {
        FileWriter fstream = null;
        try {
            fstream = new FileWriter(filePath, true);
        } catch (IOException e) {
            CustomPrinter.println(e.toString());
        }
        out = new BufferedWriter(fstream);
        i = 0;
    }

    public void end() {
        try {
            out.close();
        } catch (IOException e) {
            CustomPrinter.println(e.toString());
        }
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
            if (i++ >= 50) {
                out.flush();
                i = 0;
            }
            out.write(content);
        } catch (IOException e) {
            CustomPrinter.println("Error while writing to file: " +
                    e.getMessage());
        }
    }
}
