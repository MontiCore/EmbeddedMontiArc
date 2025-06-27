/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.cplex;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.MIQPProblem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.QPProblem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.MIQPSolverGeneratorImplementation;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.QPSolverGeneratorImplementation;
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

public class CplexSolverGeneratorImplementation implements QPSolverGeneratorImplementation, MIQPSolverGeneratorImplementation {

    // static templates
    private static final Template CALL_CPLEX_H;
    private static final Template CPLEXMAT_H;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template/optimizationSolver/cplex/");
        conf.setNumberFormat("0.####E0");
        conf.setLocale(Locale.ENGLISH);
        try {
            CALL_CPLEX_H = conf.getTemplate("CallCplexTemplate_HeaderOnly.ftl");
            CPLEXMAT_H = conf.getTemplate("CplexMat.h");
        } catch (IOException e) {
            String msg = "could not load cplex templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    // fields
    private List<String> necessaryIncludes = new ArrayList<>();

    @Override
    public String generateSolverCode(Problem optimizationProblem, List<FileContent> auxillaryFiles, EMAMBluePrintCPP bluePrint) {
        /* ToDo: Fix after Ipopt works
        String result = "";

        GeneratorCPP generator = (GeneratorCPP) bluePrint.getGenerator();
        if ((optimizationProblem instanceof QPProblem) || (optimizationProblem instanceof MIQPProblem)) {
            // create view model from problem class
            CplexViewModel vm = new CplexViewModel(optimizationProblem);
            vm.setOptions(generator.getMathOptSolverConfig().getSolverOptions());
            // set execute command
            vm.setKnownVariablesFromBluePrint(bluePrint);
            String knownVariables = ", ";
            for (String s : vm.getKnownVariables()) {
                knownVariables += s + ", ";
            }
            if (knownVariables.length() >= 2) {
                knownVariables = knownVariables.substring(0, knownVariables.length() - 2);
            }
            String objVar = vm.getObjectiveVariableName();
            if (objVar.isEmpty())
                objVar = "objectiveValue" + optimizationProblem.getId();
            result = String.format("%s::solveOptimizationProblemCplex(%s, %s%s);\n", vm.getCallSolverName(), vm.getOptimizationVariableName(), objVar, knownVariables);
            // generate templates by view model
            generateCplexTemplates(vm, auxillaryFiles);
            necessaryIncludes.add(vm.getCallSolverName());
            addCMakeDependenciesToGenerator(bluePrint);
        } else {
            Log.error(String.format("CPLEX can not solve problemes of type %s", optimizationProblem.getClass().toString()));
        }
        return result;*/
        return "";
    }

    @Override
    public List<String> getNecessaryIncludes() {
        return necessaryIncludes;
    }

    protected void generateCplexTemplates(CplexViewModel viewModel, List<FileContent> auxiliaryFiles) {

        Map<String, Object> dataForTemplate = TemplateHelper.getDataForTemplate(viewModel);

        try {
            StringWriter sw = new StringWriter();
            // add execute call
            CALL_CPLEX_H.process(dataForTemplate, sw);
            auxiliaryFiles.add(new FileContent(sw.toString(), "/" + viewModel.getCallSolverName() + ".h"));
            // add matrix type
            sw = new StringWriter();
            CPLEXMAT_H.process(dataForTemplate, sw);
            auxiliaryFiles.add(new FileContent(sw.toString(), "/" + "CplexMat.h"));
        } catch (TemplateException | IOException e) {
            Log.error("CPLEX template generation failed. ", e);
        }
    }

    protected void addCMakeDependenciesToGenerator(EMAMBluePrintCPP bluePrint) {
        Generator gen = bluePrint.getGenerator();
        if (gen instanceof GeneratorCPP) {
            CMakeConfig cmake = ((GeneratorCPP) gen).getCMakeConfig();
            List<CMakeFindModule> dependencies = getCMakeDependencies();
            for (CMakeFindModule dep : dependencies)
                cmake.addModuleDependency(dep);
            // additional commands
            cmake.addCMakeCommand("find_package(Threads REQUIRED)");
            cmake.addCMakeCommand("set(LIBS ${LIBS} ${CMAKE_THREAD_LIBS_INIT})");
            cmake.addCMakeCommand("set(LIBS ${LIBS} ${CMAKE_DL_LIBS})");
            cmake.addCMakeCommand("set( CMAKE_CXX_FLAGS  \"${CMAKE_CXX_FLAGS} -DIL_STD -Wno-deprecated\" )");
        }
    }

    public List<CMakeFindModule> getCMakeDependencies() {
        CMakeFindModule findIloCplex = new CMakeFindModule("ILOCPLEX", "ilcplex/ilocplex.h", "ilocplex", Arrays.asList("C:/Program Files/IBM/ILOG/CPLEX_Studio128/cplex/include", "/opt/ibm/ILOG/CPLEX_Studio128/cplex/include"), Arrays.asList("C:/Program Files/IBM/ILOG/CPLEX_Studio128/cplex/lib/x64_windows_vs2017/stat_mda", "/opt/ibm/ILOG/CPLEX_Studio128/cplex/lib/x86-64_linux/static_pic"), new ArrayList(), new ArrayList(), new ArrayList(), true, true, true);
        CMakeFindModule findConcert = new CMakeFindModule("CONCERT", "ilconcert/ilomodel.h", "concert", Arrays.asList("C:/Program Files/IBM/ILOG/CPLEX_Studio128/concert/include", "/opt/ibm/ILOG/CPLEX_Studio128/concert/include"), Arrays.asList("C:/Program Files/IBM/ILOG/CPLEX_Studio128/concert/lib/x64_windows_vs2017/stat_mda", "/opt/ibm/ILOG/CPLEX_Studio128/concert/lib/x86-64_linux/static_pic"), new ArrayList(), new ArrayList(), new ArrayList(), true, true, true);
        CMakeFindModule findCplex = new CMakeFindModule("CPLEX", "", "cplex", new ArrayList<String>(), Arrays.asList("C:/Program Files/IBM/ILOG/CPLEX_Studio128/cplex/lib/x64_windows_vs2017/stat_mda", "/opt/ibm/ILOG/CPLEX_Studio128/cplex/lib/x86-64_linux/static_pic"), new ArrayList(), new ArrayList(), new ArrayList(), false, true, true);
        CMakeFindModule findM = new CMakeFindModule("M", "math.h", "m", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), true, true, true);
        return Arrays.asList(findIloCplex, findConcert, findCplex, findM);
    }
}
