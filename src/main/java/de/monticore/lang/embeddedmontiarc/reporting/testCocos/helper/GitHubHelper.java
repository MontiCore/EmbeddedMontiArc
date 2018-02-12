package de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper;

import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;


public class GitHubHelper {
    public String getHTMLTagOf(File project, File file) {
        return getHTMLTagOf(project, file, getGitHubRoot(project));
    }

    public String getHTMLTagOf(File project, File file, String gitHubRoot) {
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());

        String pathFromProject = file.getAbsolutePath().substring(project.getAbsolutePath().length() + 1);
        return getHTMLLinkOf(gitHubRoot + pathFromProject, name);
    }

    public String getHTMLLinkOf(String link, String name) {
        String Name = name;
        if (link.contains("MontiSim"))
            Name = "MontiSim/" + Name;
        return "<a target=\'_blank\' href=\'" + link + "\'>" + Name + "<\\a>";
    }

    public String getGitHubRoot(File dir) {
        if (!dir.isDirectory()) Log.error("File must be a directory.");
        else {
            String[] lines = null;
            try {
                lines = execCmd("git remote show origin", dir).split("\n");
            } catch (IOException e) {
                e.printStackTrace();
            }

            String url = "";
            String branch = "";
            try {
                for (String line : lines) {
                    if (line.contains("Fetch URL"))
                        url = "https://" + line.substring(line.indexOf("git://") + "git://".length(), line.lastIndexOf(".git")) + "/";
                    if (line.contains("HEAD branch"))
                        branch = line.substring(line.indexOf("HEAD branch") + "HEAD branch: ".length());
                }
            } catch (IndexOutOfBoundsException e) {
                if (dir.getAbsolutePath().contains("EmbeddedMontiArc"))
                    url = "https://github.com/EmbeddedMontiArc/" + dir.getName() + "/";
                else if (dir.getAbsolutePath().contains("MontiSim"))
                    url = "https://github.com/MontiSim/" + dir.getName() + "/";
                branch = "master";
            }

            return url + "blob/" + branch + "/";
        }
        return "";
    }

    public static String execCmd(String cmd, File dir) throws IOException {
        java.util.Scanner s = new java.util.Scanner(Runtime.getRuntime().exec(cmd, null, dir).getInputStream()).useDelimiter("\\A");
        return s.hasNext() ? s.next() : "";
    }
}
