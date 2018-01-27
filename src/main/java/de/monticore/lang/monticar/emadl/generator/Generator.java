/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.emadl.generator;

import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import freemarker.cache.FileTemplateLoader;
import freemarker.cache.TemplateLoader;
import freemarker.template.Configuration;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;


public class Generator {

    public static final String GENERATION_PATH = "./target/";
    public static final String PYTHON_FILE_ENDING = ".py";
    public static final String CPP_FILE_ENDING = ".cpp";

    private Configuration freemarkerConfig;
    private LanguageFamilyModelLoader loader;
    private TemplateData templateData;

    public Generator() throws IOException {
        try{
            File templateDir = new File(getClass().getResource("/templates").getPath());
            TemplateLoader templateLoader = new FileTemplateLoader(templateDir);
            freemarkerConfig = createFreemarkerConfig(templateLoader);
        }
        catch(IOException e){
            Log.error("Template directory could not be found");
            throw e;
        }
    }

    public Configuration getFreemarkerConfig(){
        return freemarkerConfig;
    }

    protected LanguageFamilyModelLoader getLoader() {
        return loader;
    }

    protected TemplateData getTemplateData() {
        return templateData;
    }

    public void setFreemarkerConfig(Configuration freemarkerConfig) {
        this.freemarkerConfig = freemarkerConfig;
    }

    public void generate(Path modelPath, String qualifiedName) throws IOException, TemplateException {

        loadModel(modelPath, qualifiedName);
        checkModel();
        createTemplateData();

        TemplateController tc = new TemplateController(getTemplateData(), getFreemarkerConfig());
        FileWriter writer = getTargetFileWriter(modelPath, qualifiedName);
        /*tc.process(writer);*/

    }


    protected Configuration createFreemarkerConfig(TemplateLoader templateLoader) throws IOException {
        Configuration cfg = new Configuration(Configuration.VERSION_2_3_23);
        cfg.setTemplateLoader(templateLoader);
        cfg.setDefaultEncoding("UTF-8");
        cfg.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);
        return cfg;
    }

    protected void loadModel(Path path, String qualifiedName){
        loader = new LanguageFamilyModelLoader(path);
        loader.loadEMADLModel(qualifiedName);
    }

    protected void checkModel() {
        //EMADLCocos.createChecker().checkAll(loader.getEmadlAst());
        /*CNNArchCocos.createChecker().checkAll(loader.getArchAst());*/
        CNNTrainCocos.createChecker().checkAll(loader.getTrainAst());
    }

    protected void createTemplateData() {
        templateData = new TemplateData(loader.getGlobalScope());
        /*templateData.handle(loader.getEmadlAst());*/
    }

    protected String getTargetFileEnding() {
        if (templateData == null) {
            //Log.error("");
        }
        String fileEnding = "";
        /*switch(templateData.getTarget()) {
            case PYTHON:
                fileEnding = PYTHON_FILE_ENDING;
                break;
            case CPLUSPLUS:
                fileEnding = CPP_FILE_ENDING;
                break;
        }*/
        return fileEnding;
    }

    protected Path getTargetFilePath(Path modelPath, String qualifiedName) {
        List<String> list = Splitters.DOT.splitToList(qualifiedName);
        String targetDir = modelPath.toAbsolutePath().toString() + Joiners.DOT.join(list.subList(0, list.size() - 1));
        String targetName = list.get(list.size() - 1);
        return Paths.get(targetDir,
                GENERATION_PATH + targetName + getTargetFileEnding());
    }

    protected FileWriter getTargetFileWriter(Path modelPath, String targetName) throws IOException {
        File file = getTargetFilePath(modelPath, targetName).toFile();
        file.getParentFile().mkdirs();
        file.createNewFile();
        return new FileWriter(file);
    }


    public static void main(String[] args) throws IOException, TemplateException {
        if(args.length != 2){
            System.err.println("Two argument are required (directory path and file name (without extension))");
        }
        Path modelPath = Paths.get(args[0]).toAbsolutePath();
        String qualifiedName = args[1];

        Generator generator = new Generator();
        generator.generate(modelPath, qualifiedName);
    }
}
