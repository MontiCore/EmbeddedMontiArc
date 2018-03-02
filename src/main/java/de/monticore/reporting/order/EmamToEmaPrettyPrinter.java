package de.monticore.reporting.order;

import de.monticore.lang.monticar.helper.IndentPrinter;
import org.apache.commons.lang3.StringUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class EmamToEmaPrettyPrinter {

    public static String getEmaString(String filePath) {
        IndentPrinter ip = new IndentPrinter();

        File file = new File(filePath);

        boolean insideImplementation = false;
        boolean firstBracketFound = false;
        int bracketCount = 0;

        try(BufferedReader br = new BufferedReader(new FileReader(file))) {
            for(String line; (line = br.readLine()) != null; ) {
                if (line.contains("implementation Math")){
                    insideImplementation = true;
                }
                if(insideImplementation && bracketCount == 0 && firstBracketFound){
                    insideImplementation = false;
                }
                if(insideImplementation){
                    if(line.contains("{")) firstBracketFound = true;
                    int openingBrackets = StringUtils.countMatches(line, "{");
                    int closingBrackets = StringUtils.countMatches(line, "}");
                    bracketCount = bracketCount + openingBrackets - closingBrackets;
                } else {
                    ip.println(line);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        return ip.getContent();
    }

}
