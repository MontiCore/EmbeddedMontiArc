/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.prettyprint.IndentPrinter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;

public class MathComponentGenerator {

    public void generate(String name, String packageName,
                         Map<String, String> inPorts, Map<String, String> outPorts,
                         List<String> mathStatements, String path) {
        String component = generateAsString(name, packageName, inPorts, outPorts, mathStatements);
        writeFile(name, packageName, path, component);
    }

    public void generate(String name, String packageName,
                         Map<String, String> inPorts, Map<String, String> outPorts,
                         Map<String, String> subComponents, Map<String, String> connectors, String path) {
        String component = generateAsString(name, packageName, inPorts, outPorts, subComponents, connectors);
        writeFile(name, packageName, path, component);
    }

    public String generateAsString(String name, String packageName,
                                   Map<String, String> inPorts, Map<String, String> outPorts,
                                   List<String> mathStatements) {
        IndentPrinter printer = new IndentPrinter();
        printHeader(printer, name, packageName);
        printPorts(printer, inPorts, outPorts);
        printMathStatements(printer, mathStatements);
        printFooter(printer);

        return printer.getContent();
    }

    public String generateAsString(String name, String packageName,
                                   Map<String, String> inPorts, Map<String, String> outPorts,
                                   Map<String, String> subComponents,
                                   Map<String, String> connectors) {
        IndentPrinter printer = new IndentPrinter();
        printHeader(printer, name, packageName);
        printPorts(printer, inPorts, outPorts);
        printSubComponents(printer, subComponents);
        printConnectors(printer, connectors);
        printFooter(printer);

        return printer.getContent();
    }

    private void printHeader(IndentPrinter printer, String name, String packageName) {
        printer.println("package " + packageName + ";");
        printer.println("");
        printer.println("component " + name + " {");
        printer.indent();
    }

    private void printFooter(IndentPrinter printer) {
        printer.unindent();
        printer.println("}");
    }

    private void printPorts(IndentPrinter printer, Map<String, String> inPorts, Map<String, String> outPorts) {
        if (!inPorts.isEmpty() || !outPorts.isEmpty()) {
            printer.println("port");
            printer.indent();
            boolean first = true;
            for (Map.Entry<String, String> nameType : inPorts.entrySet()) {
                if (!first)
                    printer.print(",\n");
                printer.print("in " + nameType.getValue() + " " + nameType.getKey());
                first = false;
            }
            for (Map.Entry<String, String> nameType : outPorts.entrySet()) {
                if (!first)
                    printer.print(",\n");
                printer.print("out " + nameType.getValue() + " " + nameType.getKey());
                first = false;
            }
            printer.println(";");
            printer.unindent();
        }
    }

    private void printMathStatements(IndentPrinter printer, List<String> mathStatements) {
        if (!mathStatements.isEmpty()) {
            printer.println();
            printer.println("implementation Math {");
            printer.indent();

            for (String mathStatement : mathStatements) {
                printer.println(mathStatement + ";");
            }

            printer.unindent();
            printer.println("}");
        }
    }


    private void printSubComponents(IndentPrinter printer, Map<String, String> subComponents) {
        for (Map.Entry<String, String> subComponent : subComponents.entrySet())
            printer.println("instance " + subComponent.getValue() + " " + subComponent.getKey() + ";");
    }

    private void printConnectors(IndentPrinter printer, Map<String, String> connectors) {
        for (Map.Entry<String, String> connector : connectors.entrySet())
            printer.println("connect " + connector.getKey() + " -> " + connector.getValue() + ";");
    }

    private void writeFile(String name, String packageName, String path, String content) {
        String filePath = path + "/" + packageName.replace(".", "/");
        String fileName = filePath + "/" + name + ".emam";
        File pathFile = new File(filePath);
        if (!pathFile.exists())
            pathFile.mkdirs();

        try {
            FileWriter writer = new FileWriter(fileName);
            writer.write(content);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
