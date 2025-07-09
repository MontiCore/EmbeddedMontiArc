/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain._symboltable.LearningMethod;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 *
 */
public class CheckLearningParameterCombination implements CNNTrainASTEntryCoCo {
    private final ParameterAlgorithmMapping parameterAlgorithmMapping;

    private Set<ASTEntry> allEntries;

    private Boolean learningMethodKnown;
    private LearningMethod learningMethod;

    public CheckLearningParameterCombination() {
        allEntries = new HashSet<>();
        learningMethodKnown = false;
        parameterAlgorithmMapping = new ParameterAlgorithmMapping();
    }

    private Boolean isLearningMethodKnown() {
        return learningMethodKnown;
    }

    @Override
    public void check(ASTEntry node) {
        if (node instanceof ASTLearningMethodEntry) {
            evaluateLearningMethodEntry(node);
        } else {
            evaluateEntry(node);
        }
    }

    private void evaluateEntry(ASTEntry node) {
        allEntries.add(node);
        final boolean supervisedLearningParameter
            = parameterAlgorithmMapping.isSupervisedLearningParameter(node.getClass());
        final boolean reinforcementLearningParameter
            = parameterAlgorithmMapping.isReinforcementLearningParameter(node.getClass());
        final boolean ganLearningParameter
                = parameterAlgorithmMapping.isGANLearningParameter(node.getClass());

        assert (supervisedLearningParameter  || reinforcementLearningParameter || ganLearningParameter) :
                "Parameter " + node.getName() + " is not checkable, because it is unknown to Condition";

        if (supervisedLearningParameter && !reinforcementLearningParameter && !ganLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotSupervised(node);
        } else if(!supervisedLearningParameter && reinforcementLearningParameter && !ganLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotReinforcement(node);
        } else if(!supervisedLearningParameter && !reinforcementLearningParameter && ganLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotGAN(node);
        } else if(learningMethodKnown && learningMethod.equals(LearningMethod.REINFORCEMENT)
                && supervisedLearningParameter && !reinforcementLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotSupervised(node);
        } else if(learningMethodKnown && learningMethod.equals(LearningMethod.REINFORCEMENT)
                && ganLearningParameter && !reinforcementLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotGAN(node);
    }
}

    private void setLearningMethodOrLogErrorIfActualLearningMethodIsNotReinforcement(ASTEntry node) {
        if (isLearningMethodKnown()) {
            if (!learningMethod.equals(LearningMethod.REINFORCEMENT)) {
                Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER + " Parameter "
                                + node.getName() + " is not supported for " + this.learningMethod + " learning.",
                        node.get_SourcePositionStart());
            }
        } else {
            setLearningMethodToReinforcement();
        }
    }

    private void setLearningMethodOrLogErrorIfActualLearningMethodIsNotSupervised(ASTEntry node) {
        if (isLearningMethodKnown()) {
            if (!learningMethod.equals(LearningMethod.SUPERVISED)) {
                Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER + " Parameter "
                                + node.getName() + " is not supported for " + this.learningMethod + " learning.",
                        node.get_SourcePositionStart());
            }
        } else {
            setLearningMethodToSupervised();
        }
    }

    private void setLearningMethodOrLogErrorIfActualLearningMethodIsNotGAN(ASTEntry node) {
        if (isLearningMethodKnown()) {
            if (!learningMethod.equals(LearningMethod.GAN)) {
                Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER + " Parameter "
                                + node.getName() + " is not supported for " + this.learningMethod + " learning.",
                        node.get_SourcePositionStart());
            }
        } else {
            setLearningMethodToGAN();
        }
    }

    private void evaluateLearningMethodEntry(ASTEntry node) {
        ASTLearningMethodValue learningMethodValue = (ASTLearningMethodValue)node.getValue();
        LearningMethod evaluatedLearningMethod;
        if(learningMethodValue.isPresentReinforcement())
            evaluatedLearningMethod = LearningMethod.REINFORCEMENT;
        else if(learningMethodValue.isPresentGan())
            evaluatedLearningMethod = LearningMethod.GAN;
        else
            evaluatedLearningMethod = LearningMethod.SUPERVISED;

        if (isLearningMethodKnown()) {
            logErrorIfEvaluatedLearningMethoNotEqualToActual(node, evaluatedLearningMethod);
        } else {
            setLearningMethod(evaluatedLearningMethod);
        }
    }

    private void logErrorIfEvaluatedLearningMethoNotEqualToActual(ASTEntry node, LearningMethod evaluatedLearningMethod) {
        if (!evaluatedLearningMethod.equals(this.learningMethod)) {
            String wrongParameter = findWrongParameter(evaluatedLearningMethod);
            Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER
                    + " Parameter " + wrongParameter + " for " + evaluatedLearningMethod + ".",
                    node.get_SourcePositionStart());
        }
    }

    private String findWrongParameter(LearningMethod learningMethod) {
        List<Class> allowedParameters = getAllowedParametersByLearningMethod(learningMethod);
        for (ASTEntry entry : this.allEntries) {
            if (!allowedParameters.contains(entry.getClass())) {
                return entry.getName();
            }
        }
        return null;
    }

    private List<Class> getAllowedParametersByLearningMethod(final LearningMethod learningMethod) {
        if (learningMethod.equals(LearningMethod.REINFORCEMENT)) {
            return parameterAlgorithmMapping.getAllReinforcementParameters();
        }
        if (learningMethod.equals(LearningMethod.GAN)) {
            return parameterAlgorithmMapping.getAllGANParameters();
        }

        return parameterAlgorithmMapping.getAllSupervisedParameters();
    }


    private void setLearningMethod(final LearningMethod learningMethod) {
        if (learningMethod.equals(LearningMethod.REINFORCEMENT))
            setLearningMethodToReinforcement();
        else if (learningMethod.equals(LearningMethod.GAN))
            setLearningMethodToGAN();
        else
            setLearningMethodToSupervised();
    }

    private void setLearningMethodToSupervised() {
        this.learningMethod = LearningMethod.SUPERVISED;
        this.learningMethodKnown = true;
    }

    private void setLearningMethodToReinforcement() {
        this.learningMethod = LearningMethod.REINFORCEMENT;
        this.learningMethodKnown = true;
    }

    private void setLearningMethodToGAN() {
        this.learningMethod = LearningMethod.GAN;
        this.learningMethodKnown = true;
    }
}
