/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver.daecpp;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.loopSolver.MassMatrixViewModel;
import de.monticore.lang.monticar.generator.cpp.loopSolver.SemiExplicitFormViewModel;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.template.TemplateHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.util.Collection;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;

public class DAECPPEquationSystemGenerator {

//    private static final Template EQUATIONSYSTEM_TEMPLATE;
    private static final Template MASSMATRIX_TEMPLATE;
//    private static final Template RHS_TEMPLATE;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_29);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template/loopSolver/daecpp/");
        conf.setNumberFormat("#.################");
        conf.setLocale(Locale.ENGLISH);
        try {
//            EQUATIONSYSTEM_TEMPLATE = conf.getTemplate("EquationSystem.ftl");
            MASSMATRIX_TEMPLATE = conf.getTemplate("MassMatrix.ftl");
//            RHS_TEMPLATE = conf.getTemplate("RHS.ftl");
        } catch (IOException e) {
            String msg = "could not load cmake templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    public static Collection<FileContent> generateMassMatrix(SemiExplicitForm semiExplicitForm, String name) {
        Collection<FileContent> result = new HashSet<>();
        MassMatrixViewModel viewModel = new MassMatrixViewModel(name, semiExplicitForm);
        // map data
        Map<String, Object> dataForTemplate = TemplateHelper.getDataForTemplate(viewModel);
        // try generate file content
        try {
            StringWriter sw = new StringWriter();
//            EQUATIONSYSTEM_TEMPLATE.process(dataForTemplate, sw);
//            result.add(new FileContent(sw.toString(), String.format("/%s.h", eqs.getName())));

            sw = new StringWriter();
            MASSMATRIX_TEMPLATE.process(dataForTemplate, sw);
            result.add(new FileContent(sw.toString(), String.format("/%s_MassMatrix.h", name)));

//            sw = new StringWriter();
//            RHS_TEMPLATE.process(dataForTemplate, sw);
//            result.add(new FileContent(sw.toString(), String.format("/%s_RHS.h", name)));
        } catch (TemplateException | IOException e) {
            Log.error("EquationSystem template generation failed. ", e);
        }
        return result;
    }

}
