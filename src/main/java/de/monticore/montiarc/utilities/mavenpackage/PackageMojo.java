package de.monticore.montiarc.utilities.mavenpackage;

import de.monticore.montiarc.utilities.mavenpackage.tools.SearchFiles;
import de.monticore.montiarc.utilities.mavenpackage.tools.ZipFileCreator;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

@Mojo(name = "emam-package", defaultPhase = LifecyclePhase.PACKAGE)
public class PackageMojo extends AbstractMojo {

    @Parameter(property = "pathMain", defaultValue = "/src/main/emam/")
    private String pathMain;

    public String getPathMain() {
        return pathMain;
    }

    public void setPathMain(String path) {
        if (!path.endsWith("/")) {
            path += "/";
        }
        this.pathMain = path;
    }

    @Parameter(property = "pathOut", defaultValue = "./target/")
    private String pathOut;
    public void setPathOut(String pathOut){
        if(!pathOut.endsWith("/")){
            pathOut += "/";
        }
        this.pathOut = pathOut;
    }
    public String getPathOut(){
        return this.pathOut;
    }

    @Parameter(property = "packageName", defaultValue = "package.zip")
    private String packageName;
    public String getPackageName() {
        return packageName;
    }

    public void setPackageName(String packageName) {
        if(!packageName.endsWith(".zip")){
            packageName += ".zip";
        }
        this.packageName = packageName;
    }

    @Parameter(property = "filesToPack", defaultValue = "ema,emam,stream,struct,enum")
    private String filesToPack;
    public String getFilesToPack(){
        return filesToPack;
    }
    public void setFilesToPack(String filesToPack){
        filesToPack = filesToPack.replaceAll("\\s", "");
        this.filesToPack = filesToPack;
    }

    public String[] filesToPackAsArray(){
        return filesToPack.split(",");
    }

    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {
        Map<String, File> allFiles = this.searchFilesToPack();

        try{
            File outBase = new File(this.pathOut);
            outBase.mkdirs();
        }catch (Exception ex){
            throw new MojoFailureException("Can't create folder "+this.pathOut);
        }

        if(!ZipFileCreator.Zip(this.pathOut+this.packageName, allFiles)){
            throw new MojoExecutionException("Error while creating zip file");
        }
    }

    public Map<String,File> searchFilesToPack(){
        return SearchFiles.searchFilesMap(this.pathMain, this.filesToPackAsArray());
    }
}