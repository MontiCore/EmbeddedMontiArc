/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.ipopt;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.DNLPProblem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.NLPProblem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.NLPSolverGeneratorImplementation;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.SolverOptions;
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
 * Generates Ipopt C++ code to solve a given problem
 */
public class IpoptSolverGeneratorImplementation implements NLPSolverGeneratorImplementation {

    private static final Template CALL_IPOPT_HPP;
    private static final Template ADMAT_H;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template/optimizationSolver/ipopt/");
        conf.setNumberFormat("0.####E0");
        conf.setLocale(Locale.ENGLISH);
        try {
            CALL_IPOPT_HPP = conf.getTemplate("CallIpoptTemplate_HeaderOnly.ftl");
            ADMAT_H = conf.getTemplate("ADMat.h");
        } catch (IOException e) {
            String msg = "could not load ipopt templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    // fields
    private List<String> necessaryIncludes = new ArrayList<>();

    // constructor
    public IpoptSolverGeneratorImplementation() {

    }

    @Override
    public String generateSolverCode(Problem optimizationProblem, List<FileContent> auxillaryFiles, EMAMBluePrintCPP bluePrint) {
        String result = "";
        //optimizationProblem.
//        GeneratorEMAMOpt2CPP generator = (GeneratorEMAMOpt2CPP) bluePrint.getGenerator();
        GeneratorCPP generator = (GeneratorCPP) bluePrint.getGenerator();
        if ((optimizationProblem instanceof NLPProblem) || (optimizationProblem instanceof DNLPProblem)) {
            // create view model from problem class
            IpoptViewModel vm = new IpoptViewModel(optimizationProblem);
            SolverOptions options = SolverOptions.getIpoptDefaultOptions();
            options.putAll(generator.getMathOptSolverConfig().getSolverOptions()); //options.putAll(generator.getSolverOptions());
            // retape if DNLP
            if (optimizationProblem instanceof DNLPProblem)
                options.put("Retape", "true");
            vm.setOptions(options);
            // set execute command
            vm.setExternalVariablesFromBluePrint(bluePrint);
            result = String.format("%s::solveOptimizationProblemIpOpt(%s);\n", vm.getCallSolverName(),vm.getIpoptSolverFunctionCallParameters());
            // generate templates by view model
            generateIpoptTemplates(vm, auxillaryFiles);
            necessaryIncludes.add(vm.getCallSolverName());
            addCMakeDependenciesToGenerator(bluePrint);
        } else {
            Log.error(String.format("Ipopt can not solve problemes of type %s", optimizationProblem.getClass().toString()));
        }
        return result;
    }

    // methods

    @Override
    public List<String> getNecessaryIncludes() {
        return necessaryIncludes;
    }

    protected void generateIpoptTemplates(IpoptViewModel viewModel, List<FileContent> auxillaryFiles) {

        Map<String, Object> dataForTemplate = TemplateHelper.getDataForTemplate(viewModel);

        try {
            StringWriter sw = new StringWriter();
            // add execute call
            CALL_IPOPT_HPP.process(dataForTemplate, sw);
            auxillaryFiles.add(new FileContent(sw.toString(), "/" + viewModel.getCallSolverName() + ".h"));
            // add matrix type
            sw = new StringWriter();
            ADMAT_H.process(dataForTemplate, sw);
            auxillaryFiles.add(new FileContent(sw.toString(), "/" + "ADMat.h"));
        } catch (TemplateException | IOException e) {
            Log.error("Ipopt template generation failed. ", e);
        }
    }

    protected void addCMakeDependenciesToGenerator(EMAMBluePrintCPP bluePrint) {
        Generator gen = bluePrint.getGenerator();
        if (gen instanceof GeneratorCPP) {
            List<CMakeFindModule> dependencies = getCMakeDependencies();
            for (CMakeFindModule dep : dependencies)
                ((GeneratorCPP) gen).getCMakeConfig().addModuleDependency(dep);
        }
//        if (gen instanceof GeneratorEMAMOpt2CPP) {
//            List<CMakeFindModule> dependencies = getCMakeDependencies();
//            for (CMakeFindModule dep : dependencies)
//                ((GeneratorEMAMOpt2CPP) gen).getCMakeConfig().addModuleDependency(dep);
//        }
    }

    public List<CMakeFindModule> getCMakeDependencies() {
        CMakeFindModule findCPPAD = new CMakeFindModule("CPPAD", "cppad/ipopt/solve.hpp", "", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), true, false, true);
        CMakeFindModule findIPOpt = new CMakeFindModule("Ipopt", "coin/IpNLP.hpp", "ipopt", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), true, true, true);
        //CMakeFindModule findCoinMumps = new CMakeFindModule("CoinMumps", "", "coinmumps", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), false, true, true);
        //CMakeFindModule findCoinLapack = new CMakeFindModule("CoinLapack", "", "coinlapack", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), false, true, true);
        //CMakeFindModule findCoinBlas = new CMakeFindModule("CoinBlas", "", "coinblas", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), false, true, true);
        //CMakeFindModule findCoinMetis = new CMakeFindModule("CoinMetis", "", "coinmetis", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), false, true, true);
        //CMakeFindModule findGfortran = new CMakeFindModule("GFortran", "", "gfortran", new ArrayList<String>(), new ArrayList<String>(), new ArrayList(), new ArrayList(), new ArrayList(), false, true, true);
        return Arrays.asList(findCPPAD, findIPOpt);
    }

}
