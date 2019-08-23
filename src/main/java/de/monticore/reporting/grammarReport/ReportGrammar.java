/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.grammarReport;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.Main;
import de.monticore.reporting.tools.GitLabHelper;
import de.monticore.reporting.tools.SearchFiles;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;


public class ReportGrammar {

    public static void reportGrammars(Main.ReportContext context, String outputFile, boolean merge) {
        (new ReportGrammar()).reportGrammars_(context, outputFile, merge);
    }

    public void reportGrammars_(Main.ReportContext context, String outputFile, boolean merge) {
        List<File> files = SearchFiles.searchFiles(new File(context.getProjectRoot()), "mc4", "mc5");
        List<GrammarInfo> grammars = new LinkedList<>();

        for(File file: files) {
            String name = file.getAbsolutePath().substring(new File(context.getProjectRoot()).
                    getAbsolutePath().length() + 1)
                    .replace(".mc4","").replace(".mc5", "")
                    .replace("\\","/");
            grammars.add(new GrammarInfo(file, name));
        }

        printGrammars(grammars, context, outputFile, merge);
    }

    private void printGrammars(List<GrammarInfo> grammars, Main.ReportContext context, String outputFile, boolean merge){
        if (grammars.size() == 0) return;
        if (merge) {
            try {
                String first = FileUtils.readFileToString(new File(outputFile));
                first = first.substring(0, first.length() - 3);
                String str = first + ",\n" + getPrintString(grammars, context, merge);
                FileUtils.writeStringToFile(new File(outputFile),
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            try {
                FileUtils.writeStringToFile(new File(outputFile),
                        getPrintString(grammars, context, merge));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private String getPrintString(List<GrammarInfo> grammars, Main.ReportContext context, boolean merge){
        String[] names = {
                "\"Root\"",
                "\"Name\""
        };
        IndentPrinter ip = new IndentPrinter();
        if (!merge)
            ip.println("[");
        ip.indent();

        boolean first = true;
        for (GrammarInfo grammar : grammars) {
            if (grammar == null) continue;
            int i = 0;

            if (!first)
                ip.print(",\n");
            else
                first = false;

            ip.println("{");
            ip.indent();
            ip.println(names[i++] + ": \"" + getComputedRootName(context.getProjectRoot(), grammar) + "\",");
            ip.println(names[i++] + ": \"" + getGitLabLink(grammar, context.getProjectRoot()) + "\"");
            ip.unindent();
            ip.print("}");
        }
        ip.println();
        ip.unindent();
        ip.println("]");

        return ip.getContent();

    }

    private static String getComputedRootName(String root, GrammarInfo grammar){
        String rootName = (new File(root)).getName().replace("\\","/");
        if(rootName.equals("MontiCore")){
//            String projectName = grammar.file.getAbsolutePath().substring(root.length() + 1).replace("\\","/");
//            projectName = projectName.substring(0,projectName.indexOf("/", projectName.indexOf("/") + 1));
//            if(projectName.contains("monticore-grammar") && grammar.file.getAbsolutePath().contains("src\\main"))
            if(grammar.file.getAbsolutePath().contains("monticore-grammar"))
                return rootName + "&nbsp;&ndash;&nbsp;MainGrammars";
            else
                return rootName + "&nbsp;&ndash;&nbsp;Test";
        }
        return rootName;
    }

    private static String getGitLabLink(GrammarInfo grammar, String root) {
        File project = new File(root + "/" + grammar.name.substring(0, grammar.name.indexOf("/")));
        String ghLink = GitLabHelper.getHTMLTagOf(project, grammar.file);

        String htmlTag = "<a class='ghLink' href='" + ghLink + "' target='_blank' rel='noopener'>"
                + computeName(grammar) + "</a>";

        return ghLink.replace("\\","/");
    }

    private static String computeName(GrammarInfo grammar){
        String name = grammar.name.
                replace("src/main/grammar/","").
                replace("src/test/grammar/","").
                replace("src/main/grammars/","").
                replace("src/test/grammars/","").
                replace("src/main/resources/","").
                replace("src/test/resources/","").
                replace("src/main/","").
                replace("src/test/","");

        return name;
    }

    private class GrammarInfo {
        File file;
        String name;

        GrammarInfo(File file, String name){
            this.file = file;
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

}
