package de.monticore.reporting.grammarReport;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.Main;
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
            String ideLink = "";
            grammars.add(new GrammarInfo(file, name, ideLink));
        }

        printGrammars(grammars, context, outputFile, merge);
    }

    private String generateIDELink(File file, String zipName, File root){
        String zipName_ = zipName.substring(0, zipName.lastIndexOf("."));
        String urlToZip = "https://raw.githubusercontent.com/EmbeddedMontiArc/reporting/gh-pages/" + zipName;;
        String path = file.getAbsolutePath().substring(root.getAbsolutePath().length() + 1);
        String displayName = path.replace("/",".").replace("\\",".");


        return ("\"<a target='_blank' href='onlineIDE/api/load.html?mountPoint=EmbeddedMontiArc/reporting/" + zipName_ + "&url="
                + urlToZip + "&openFile=/" + path + "'>" +
                "<img border='0' alt='" + displayName + "' src='images/favicon.ico' class='onlineIDEImage'>" +
                "</a>\"").replace("\\", "/");
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
                "\"Name\"",
                "\"OnlineIDE\""
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
            ip.println(names[i++] + ": \"" + getGithubLink(grammar, context.getProjectRoot()) + "\",");
            ip.println(names[i++] + ": " + grammar.ideLink);
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

    private static String getGithubLink(GrammarInfo grammar, String root) {
        String rootName = (new File(root)).getName();
        String projectName = grammar.file.getAbsolutePath().substring(root.length() + 1).replace("\\","/");
        projectName = projectName.substring(0,projectName.indexOf("/"));
        String ghLink = "https://github.com/" + rootName + "/" + projectName
                + "/blob/master/"
                + grammar.file.getAbsolutePath().substring(
                root.length() + projectName.length() + 1)
                .replace("\\","/");
        ghLink = ghLink.replace("\\","/");

        String htmlTag = "<a class='ghLink' href='" + ghLink + "' target='_blank' rel='noopener'>"
                + computeName(grammar) + "</a>";

        return htmlTag;
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
        String ideLink;
        String name;

        GrammarInfo(File file, String name, String ideLink){
            this.file = file;
            this.name = name;
            this.ideLink = ideLink;
        }

        public String toString() {
            return name;
        }
    }

}
