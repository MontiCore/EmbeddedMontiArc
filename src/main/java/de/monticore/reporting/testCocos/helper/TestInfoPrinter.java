package de.monticore.reporting.testCocos.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class TestInfoPrinter {

    public static void printInfo(List<CheckCoCoResult> testResults, String infoPath, boolean merge){
        if (testResults.size() == 0) return;
        if (merge) {
            try {
                String first = FileUtils.readFileToString(new File(infoPath));
                first = first.substring(0, first.length() - 3);
                String str = first + ",\n" + getInfo(testResults, merge);
                FileUtils.writeStringToFile(new File(infoPath),
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            try {
                FileUtils.writeStringToFile(new File(infoPath),
                        getInfo(testResults, merge));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private static String getInfo(List<CheckCoCoResult> testResults,  boolean merge){
        ValidInfo info = getValidInfo(testResults);

        char c = 160;

        IndentPrinter ip = new IndentPrinter();
        if (!merge)
            ip.println("{");
        ip.indent();

        boolean first = true;
        int i = 0;

        if (!first)
            ip.print(",\n");
        else
            first = false;

        ip.println("\"" + info.root + "\": {");
        ip.indent();
        ip.println("\"Number\": \"" + info.number + "\",");
        ip.println("\"Valid\": \"" + info.valid + "\",");
        ip.println("\"Invalid\": \"" + info.invalid + "\"");
        ip.unindent();

        ip.print("},");
        ip.println();

        ip.println("\"" + c + "Errored_" + info.root + "\": {");
        ip.indent();
        ip.println("\"Number\": \"" + info.errored + "\",");
        ip.println("\"Valid\": \"" + 0 + "\",");
        ip.println("\"Invalid\": \"" + info.errored + "\"");
        ip.unindent();

        ip.print("}");
        ip.println();
        ip.unindent();
        ip.println("}");

        return ip.getContent();
    }

    private static ValidInfo getValidInfo(List<CheckCoCoResult> testResults){
        TestInfoPrinter tif = new TestInfoPrinter();
        return tif.getValidInfo_(testResults);
    }

    private ValidInfo getValidInfo_(List<CheckCoCoResult> testResults){
        ValidInfo info = new ValidInfo();
        String root = "";
        int number = 0;
        int valid = 0;
        int invalid = 0;
        int errored = 0;


        for(CheckCoCoResult testResult: testResults){
            number++;
            if(root.equals(""))
                root = testResult.getRootFile().getName();
            if(testResult.isValid())
                valid++;
            else
                invalid++;
            if(testResult.getParsed() != 1 || testResult.getResolved() != 1)
                errored++;
        }
        info.root = root;
        info.number = number;
        info.valid = valid;
        info.invalid = invalid;
        info.errored = errored;

        return info;
    }

    public class ValidInfo {
        public String root;
        public int number;
        public int valid;
        public int invalid;
        public int errored;
    }
}
