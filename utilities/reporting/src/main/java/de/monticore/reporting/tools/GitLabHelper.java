/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.tools;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.exec.PumpStreamHandler;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;


public class GitLabHelper {
    public static String getHTMLTagOf(File project, File file) {
        return getHTMLTagOf(project, file, getGitLabRoot(project));
    }

    public static String getHTMLTagOf(File project, File file, String gitHubRoot) {
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());

        String pathFromProject = file.getAbsolutePath().substring(project.getAbsolutePath().length() + 1);
        return getHTMLLinkOf(gitHubRoot + pathFromProject, name);
    }

    public static String getHTMLLinkOf(String link, String name) {
        return "<a target=\'_blank\' href=\'" + link + "\'>" + name + "<\\a>";
    }

    public static String getGitLabRoot(File dir) {
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
            for (String line : lines) {
                if (line.contains("Fetch URL")) {
                    url = line.substring(line.lastIndexOf(": ") + 2) + "/";
                    url = "https://" + url.substring(url.indexOf("@git") + 1);
                }
                if (line.contains("HEAD branch"))
                    branch = line.substring(line.indexOf("HEAD branch") + "HEAD branch: ".length());
            }

            return url.replace(".git", "") + "blob/" + branch + "/";
        }
        return "";
    }

    public static String execCmd(String cmd, File dir) throws IOException {
        java.util.Scanner s = new java.util.Scanner(Runtime.getRuntime().exec(cmd, null, dir).getInputStream()).useDelimiter("\\A");
        return s.hasNext() ? s.next() : "";
    }
}
