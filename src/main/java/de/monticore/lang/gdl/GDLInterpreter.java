package de.monticore.lang.gdl;

import org.sosy_lab.common.ShutdownManager;
import org.sosy_lab.common.configuration.Configuration;
import org.sosy_lab.common.configuration.InvalidConfigurationException;
import org.sosy_lab.common.log.BasicLogManager;
import org.sosy_lab.common.log.LogManager;
import org.sosy_lab.java_smt.SolverContextFactory;
import org.sosy_lab.java_smt.SolverContextFactory.Solvers;
import org.sosy_lab.java_smt.api.ArrayFormulaManager;
import org.sosy_lab.java_smt.api.BooleanFormula;
import org.sosy_lab.java_smt.api.BooleanFormulaManager;
import org.sosy_lab.java_smt.api.IntegerFormulaManager;
import org.sosy_lab.java_smt.api.Model;
import org.sosy_lab.java_smt.api.ProverEnvironment;
import org.sosy_lab.java_smt.api.SolverContext;
import org.sosy_lab.java_smt.api.SolverException;
import org.sosy_lab.java_smt.api.Model.ValueAssignment;
import org.sosy_lab.java_smt.api.NumeralFormula.IntegerFormula;
import org.sosy_lab.java_smt.api.SolverContext.ProverOptions;

public class GDLInterpreter {
    
    public static void main(String[] args) throws InvalidConfigurationException, InterruptedException, SolverException {
        Configuration config = Configuration.defaultConfiguration();
        LogManager logger = BasicLogManager.create(config);
        ShutdownManager shutdown = ShutdownManager.create();
        
        SolverContext context = SolverContextFactory.createSolverContext(config, logger, shutdown.getNotifier(), Solvers.SMTINTERPOL);

        ArrayFormulaManager amgr = context.getFormulaManager().getArrayFormulaManager();
        IntegerFormulaManager imgr = context.getFormulaManager().getIntegerFormulaManager();
        BooleanFormulaManager bmgr = context.getFormulaManager().getBooleanFormulaManager();

        IntegerFormula x = imgr.makeVariable("x");
        IntegerFormula y = imgr.makeVariable("y");
        IntegerFormula one = imgr.makeNumber(1);
        IntegerFormula two = imgr.makeNumber(2);

        BooleanFormula f = imgr.equal(x, one);
        BooleanFormula g = imgr.equal(y, two);
        BooleanFormula prop = bmgr.and(f, g);

        ProverEnvironment prover = context.newProverEnvironment(ProverOptions.GENERATE_MODELS);
        prover.addConstraint(imgr.equal(x, y));

        boolean isUnsat = prover.isUnsat();
        if (isUnsat) {
            System.out.println("UNSAT");
        } else {
            Model model = prover.getModel();
            System.out.printf("SAT with x = %s, y = %s\n", model.evaluate(x), model.evaluate(y));
            System.out.println(model.evaluate(prop));

            for (ValueAssignment va: prover.getModelAssignments()) {
                System.out.println(va);
            }

        }
    }

}
