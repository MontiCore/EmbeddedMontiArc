/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.template.TemplateHelper;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.util.*;

/**
 * Class to configure cmake file generation.
 * Is responsible for generating the cmake files from freemarker templates.
 * CMake dependencies to other modules can be added as Find Package file.
 * See also https://cmake.org/cmake/help/v3.8/command/find_package.html?highlight=i
 */
public class CMakeConfig {

    private static final Template CMAKE_LISTS_CPP;
    private static final Template CMAKE_FIND_PACKAGE;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template/cmake/");
        conf.setNumberFormat("#.################");
        conf.setLocale(Locale.ENGLISH);
        try {
            CMAKE_LISTS_CPP = conf.getTemplate("CMakeListsCppTemplate.ftl");
            CMAKE_FIND_PACKAGE = conf.getTemplate("CMakeFindPackageTemplate.ftl");
        } catch (IOException e) {
            String msg = "could not load cmake templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    // fields

    private CMakeListsCPPViewModel cMakeListsViewModel = new CMakeListsCPPViewModel();

    private LinkedHashSet<CMakeFindModule> moduleList = new LinkedHashSet<>();

    private List<String> cmakeCommandList = new ArrayList<>();

    private List<String> cmakeCommandListEnd = new ArrayList<>();

    private List<String> cmakeLibraryLinkageList = new ArrayList<>();

    // constructor
    public CMakeConfig(String compName) {
        cMakeListsViewModel.setCompName(compName);
        cMakeListsViewModel.setModuleDependencies(moduleList);
        configureCMakeListsViewModel();
    }

    // methods
    protected void configureCMakeListsViewModel() {
        cMakeListsViewModel.setCmakeLibraryLinkageList(cmakeLibraryLinkageList);
        cMakeListsViewModel.setCmakeCommandList(cmakeCommandList);
        cMakeListsViewModel.setCmakeCommandListEnd(cmakeCommandListEnd);
    }

    public List<FileContent> generateCMakeFiles() {
        List<FileContent> files = new ArrayList<FileContent>();
        // generate CMakeLists.txt
        files.add(generateCMakeLists());
        // generate ${component.name}.cpp
        files.add(generateComponentCpp());
        // generate FindModule.cmake's
        for (CMakeFindModule module : moduleList) {
            files.add(generateCMakeFindPackage(module));
        }
        return files;
    }

    private FileContent generateComponentCpp() {
        FileContent result = new FileContent();
        String compName = cMakeListsViewModel.getCompName().replace(".", "_");
        result.setFileName(compName + ".cpp");
        result.setFileContent("#include \"" + compName + ".h\"");
        return result;
    }

    private FileContent generateCMakeFindPackage(CMakeFindModule module) {
        FileContent result = null;
        // map data
        Map<String, Object> dataForTemplate = TemplateHelper.getDataForTemplate(module);
        // try generate file content
        try {
            StringWriter sw = new StringWriter();
            CMAKE_FIND_PACKAGE.process(dataForTemplate, sw);
            result = (new FileContent(sw.toString(), "/cmake/" + module.getFileName()));
        } catch (TemplateException | IOException e) {
            Log.error("CMakeFindPackage template generation failed. ", e);
        }
        return result;
    }

    public FileContent generateCMakeLists() {
        FileContent result = null;
        //configureCMakeListsViewModel();
        // map data
        Map<String, Object> dataForTemplate = TemplateHelper.getDataForTemplate(cMakeListsViewModel);
        // try generate file content
        try {
            StringWriter sw = new StringWriter();
            CMAKE_LISTS_CPP.process(dataForTemplate, sw);
            result = (new FileContent(sw.toString(), "/" + cMakeListsViewModel.CMAKELISTS_FILE_NAME));
        } catch (TemplateException | IOException e) {
            Log.error("CMakeLists template generation failed. ", e);
        }
        return result;
    }

    public void addModuleDependency(CMakeFindModule module) {
        if (!moduleList.contains(module))
            moduleList.add(module);
    }

    public CMakeListsCPPViewModel getCMakeListsViewModel() {
        return cMakeListsViewModel;
    }

    /**
     * Adds an arbitrary cmake command before target settings
     *
     * @param cmd some valid cmake command as string
     */
    public void addCMakeCommand(String cmd) {
        if (!cmakeCommandList.contains(cmd))
            cmakeCommandList.add(cmd);
    }

    /**
     * Adds an arbitrary cmake command at the end of the file
     *
     * @param cmd some valid cmake command as string
     */
    public void addCMakeCommandEnd(String cmd) {
        if (!cmakeCommandListEnd.contains(cmd))
            cmakeCommandListEnd.add(cmd);
    }


    public void addCmakeLibraryLinkage(String cmakeLibraryLinkage) {
        if (!cmakeLibraryLinkageList.contains(cmakeLibraryLinkage))
            this.cmakeLibraryLinkageList.add(cmakeLibraryLinkage);
    }


}
