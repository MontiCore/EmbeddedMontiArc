/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import org.apache.commons.io.FileUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class RewriteWithoutArray {

    public static void rewrite(String src, String dest){
        try {
            FileUtils.write(new File(dest), getWithoutArrayString(src));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public static String getWithoutArrayString(String filePath) {
        IndentPrinter ip = new IndentPrinter();

        File file = new File(filePath);

        Map<String, Integer> arraySizes = new HashMap<>();

        try(BufferedReader br = new BufferedReader(new FileReader(file))) {
            for(String line; (line = br.readLine()) != null; ) {
                if (line.contains("in ") || line.contains("out ") || line.contains("instance ") ||
                        line.contains("in\t") || line.contains("out\t") || line.contains("instance\t")){
                    String name = null;
                    String arraySizeString = null;
                    String rest = null;
                    String lineEnding = null;
                    Pattern pattern = Pattern.compile("(.*\\s+(\\S+))\\s*\\[\\s*(\\d+)\\s*\\]\\s*(;|,).*");
                    Matcher matcher = pattern.matcher(line);
                    while (matcher.find()) {
                        rest = matcher.group(1);
                        name = matcher.group(2);
                        arraySizeString = matcher.group(3);
                        lineEnding = matcher.group(4);
                    }
                    if(arraySizeString != null) {
                        boolean isSemicolon = lineEnding.equals(";");
                        boolean isInstance = line.contains("instance ") || line.contains("instance\t");
                        int arraySize = Integer.parseInt(arraySizeString);
                        arraySizes.put(name, arraySize);
                        for(int i = 0; i < arraySize; i++) {
                            if(isSemicolon && i == arraySize - 1 || isInstance)
                                ip.println(rest + (i+1) + ";");
                            else
                                ip.println(rest + (i+1) + ",");
                        }
                    }
                    else {
                        ip.println(line);
                    }
                } else if(!arraySizes.isEmpty()){
                    for(String name: arraySizes.keySet()){
                        if(line.contains("connect") && line.contains(name)){
                            if(Pattern.compile(".*" + name + ".*\\[\\s*\\d+\\s*\\].*").matcher(line).find()){
                                ip.println(line.replaceAll("\\s*\\[\\s*(\\d+)\\s*\\]","$1"));
                            } else if(Pattern.compile(".*" + name + ".*\\[\\s*:\\s*\\].*").matcher(line).find()){
                                for(int i = 0; i < arraySizes.get(name); i++)
                                    ip.println(line.replaceAll("\\s*\\[\\s*:\\s*\\]","" + (i+1)));
                            }
                            break;
                        } else
                            ip.println(line);
                    }
                }
                else
                    ip.println(line);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        return ip.getContent();
    }
}
