package de.monitcore.lang.monticar.utilities.middleware;

import de.monitcore.lang.monticar.utilities.StreamTestExecuteMojo;
import de.monitcore.lang.monticar.utilities.tools.ChecksumChecker;
import de.monitcore.lang.monticar.utilities.tools.LogToFile;
import de.monitcore.lang.monticar.utilities.tools.SearchFiles;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.middleware.DistributedTargetGenerator;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.ODVGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Checks if MiddlewareRootModels are available and
 * generates cpp / roscpp / odv code for the given emam models
 */
@Mojo(name = "streamtest-middleare-generate")
public class MiddlewareGeneratorMojo extends MiddlewareMojoBase {



    //<editor-fold desc="Execution">

    @Override
    protected void preExecution() throws MojoExecutionException, MojoFailureException {
//        super.preExecution();
//        if(middlewareGenerator == MiddlewareGenerator.odv){
        if(middlewareGenerator.contains(MiddlewareGenerator.odv)){
            logError("ODV is not yet implemented!");
            throw new MojoExecutionException("ODV is not yet implemented!");
        }

        mkdir(Paths.get(this.getPathMiddlewareOut(), mojoDirectory).toString());
        if(myLog == null) {
            myLog = LogToFile.init();
        }
        myLog.setLogFile(Paths.get(this.getPathMiddlewareOut(), mojoDirectory, this.MojoName()+".log").toString());
        myLog.clear();




        if(runStreamTestBefore){
            logInfo("Running Streamtest Mojos before generating Middleware!");
            super.preExecution();
            StreamTestExecuteMojo stem = new StreamTestExecuteMojo();
            this.copyPropertiesAndParametersTo(stem);
            stem.execute();
        }
    }

    @Override
    protected void mainExecution() throws MojoExecutionException, MojoFailureException {

        //Remove old hasfiles
        File fmain = hashFileMain();
        if(fmain.exists()){
            fmain.delete();
        }
        File ftest = hashFileTest();
        if(ftest.exists()){
            ftest.delete();
        }
        File finput = hashFileInputs();
        if(finput.exists()){
            finput.delete();
        }

        Scope scope = getScope();
        TaggingResolver tagging = this.getTaggingResolver();
        List<String> modelsInError = new ArrayList<>();
        logInfo("Generate Middleware:");
        for (String model : this.middlewareRootModels){
            logInfo(" - "+model);

            Optional<EMAComponentInstanceSymbol> ecis = scope.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND);
            if(!ecis.isPresent()){
                logError("   -> Can't resolve ExpandedComponentInstanceSymbol for "+model);
                modelsInError.add(model);
                continue;
            }

            DistributedTargetGenerator generator = new DistributedTargetGenerator();

            String modelDirName = model.replace(".", "_");
            generator.setGenerationTargetPath(Paths.get(pathMiddlewareOut, modelDirName).toString());



            if(this.middlewareGenerator.contains(MiddlewareGenerator.cpp)) {
                logInfo("   -> Adding cpp generator");
                generator.add(new CPPGenImpl(this.getPathTmpOutEMAM()), "cpp");
            }
            if(this.middlewareGenerator.contains(MiddlewareGenerator.roscpp)){
                logInfo("   -> Adding roscpp generator");
                generator.add(new RosCppGenImpl(), "roscpp");
                RosToEmamTagSchema.registerTagTypes(tagging);
                TagHelper.resolveTags(tagging, ecis.get());
            }
            if (this.middlewareGenerator.contains(MiddlewareGenerator.odv)) {
                logInfo("   -> Adding odv generator");
                generator.add(new ODVGenImpl(), "odv");
            }



            try {
                generator.generate(ecis.get(), tagging);
                logInfo("   -> Success");
                logInfo("   -> Path: "+Paths.get(pathMiddlewareOut, modelDirName).toAbsolutePath().toString());
            } catch (IOException e) {
                e.printStackTrace();
                throw new MojoExecutionException("Failed to generate middleware for "+model);
            }
        }

        if(modelsInError.size() > 0){
            throw new MojoExecutionException("Models "+String.join(", ", modelsInError)+" could not resolved to ExpandedComponentInstanceSymbol!");
        }

    }

    @Override
    protected void postExecution() throws MojoExecutionException {
//        super.postExecution();
        String mainHash = SearchFiles.hashDirFiles(this.getPathMain());
        String testHash = SearchFiles.hashDirFiles(this.getPathTest());
        String inputHash = getInputsHash();
        try {
            FileUtils.write(hashFileMain(), mainHash, false);
            FileUtils.write(hashFileTest(), testHash, false);
            FileUtils.write(hashFileInputs(), inputHash, false);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Failed to create hash files for "+MojoName());
        }
    }

    @Override
    protected boolean checkForExecution() throws MojoExecutionException {
        if(middlewareRootModels == null || middlewareRootModels.size() == 0){
            return false;
        }

        File hmain = hashFileMain();
        File htest = hashFileTest();
        File hinput = hashFileInputs();

        if(!hmain.exists() || !htest.exists() || !hinput.exists()){
            logInfo("Execution necessary: Hashfiles not found.");
            return true;
        }

        String oldMainHash = "",oldTestHash = "", oldInputHash = "";

        String newMainHash = SearchFiles.hashDirFiles(this.getPathMain());
        String newTestHash = SearchFiles.hashDirFiles(this.getPathTest());
        String newInputHash = getInputsHash();
        try {
            oldMainHash = FileUtils.readFileToString(hmain);
            oldTestHash = FileUtils.readFileToString(htest);
            oldInputHash = FileUtils.readFileToString(hinput);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Can't read old hash files");
        }

        if(newMainHash.equalsIgnoreCase(oldMainHash) && newTestHash.equalsIgnoreCase(oldTestHash) &&
                newInputHash.equalsIgnoreCase(oldInputHash)){
            logInfo("Execution of "+this.MojoName().toUpperCase()+" not necessary. No input files (or configuration) changed.");
            return false;
        }

        logInfo("Execution necessary: One or more input files chaned.");
        return true;

    }

    @Override
    protected String MojoName() {
        return "streamtest-middleare-generate";
    }

//</editor-fold>

    //<editor-fold desc="Hashfiles">

    protected File hashFileMain(){
        return Paths.get(this.pathMiddlewareOut, mojoDirectory, this.MojoName(), "Main.txt").toFile();
    }

    protected File hashFileTest(){
        return Paths.get(this.pathMiddlewareOut, mojoDirectory, this.MojoName(), "Test.txt").toFile();
    }

    protected File hashFileInputs(){
        return Paths.get(this.pathMiddlewareOut, mojoDirectory, this.MojoName(), "Inputs.txt").toFile();
    }

    //</editor-fold>

    protected String getInputsHash(){
        StringBuilder sb = new StringBuilder();
        this.middlewareRootModels.forEach(mr -> {
            sb.append(ChecksumChecker.getChecksumForStringMD5(mr));
        });

        this.middlewareGenerator.forEach(mg -> {
            sb.append(ChecksumChecker.getChecksumForStringMD5(mg.name()));
        });

        return sb.toString();
    }

}
