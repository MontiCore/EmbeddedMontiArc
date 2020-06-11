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
        String filePath = path + "/" + packageName.replace(".","/");
        String fileName = filePath + "/" + name + ".emam";
        File pathFile = new File(filePath);
        if (!pathFile.exists())
            pathFile.mkdirs();

        try {
            FileWriter writer = new FileWriter(fileName);
            writer.write(component);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public String generateAsString(String name, String packageName,
                                   Map<String, String> inPorts, Map<String, String> outPorts,
                                   List<String> mathStatements) {
        IndentPrinter printer = new IndentPrinter();
        printer.println("package " + packageName + ";");
        printer.println("");
        printer.println("component " + name + " {");
        printer.indent();
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
        printer.println(";\n");
        printer.unindent();

        printer.println("implementation Math {");
        printer.indent();

        for (String mathStatement : mathStatements) {
            printer.println(mathStatement + ";");
        }

        printer.unindent();
        printer.println("}");

        printer.unindent();
        printer.println("}");


        return printer.getContent();
    }
}
