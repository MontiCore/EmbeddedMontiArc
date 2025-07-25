/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Optional;

import static de.monticore.lang.monticar.cnnarch.helper.ErrorCodes.ILLEGAL_ASSIGNMENT;

public enum Constraints {
    NUMBER {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isNumber();
        }
        @Override
        public String msgString() {
            return "a number";
        }
    },
    INTEGER {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isInt().get();
        }
        @Override
        public String msgString() {
            return "an integer";
        }
    },
    BOOLEAN {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isBoolean();
        }
        @Override
        public String msgString() {
            return "a boolean";
        }
    },
    STRING {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isString();
        }
        @Override
        public String msgString() {
            return "a string";
        }
    },
    PATH_TAG_OR_PATH {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isString() || exp.isStringTag();
        }
        @Override
        public String msgString() {
            return "a path tag or a path string";
        }
    },
    TUPLE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isTuple();
        }
        @Override
        public String msgString() {
            return "a tuple";
        }
    },
    INTEGER_TUPLE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isIntTuple().get();
        }
        @Override
        public String msgString() {
            return "a tuple of integers";
        }
    },
    INTEGER_OR_INTEGER_TUPLE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            return exp.isInt().get() || exp.isIntTuple().get();
        }
        @Override
        public String msgString() {
            return "an integer or tuple of integers";
        }
    },
    POSITIVE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getDoubleValue().isPresent()){
                return exp.getDoubleValue().get() > 0;
            }
            else if (exp.getDoubleTupleValues().isPresent()){
                boolean isPositive = true;
                for (double value : exp.getDoubleTupleValues().get()){
                    if (value <= 0){
                        isPositive = false;
                    }
                }
                return isPositive;
            }
            return false;
        }
        @Override
        public String msgString() {
            return "a positive number";
        }
    },
    POSITIVE_OR_MINUS_ONE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getDoubleValue().isPresent()){
                return exp.getDoubleValue().get() > 0 || exp.getDoubleValue().get() == -1;
            }
            else if (exp.getDoubleTupleValues().isPresent()){
                boolean isPositive = true;
                for (double value : exp.getDoubleTupleValues().get()){
                    if (value < -1 || value == 0){
                        isPositive = false;
                    }
                }
                return isPositive;
            }
            return false;
        }
        @Override
        public String msgString() {
            return "a positive number";
        }
    },
    NON_NEGATIVE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getDoubleValue().isPresent()){
                return exp.getDoubleValue().get() >= 0;
            }
            else if (exp.getDoubleTupleValues().isPresent()){
                boolean isNonNegative = true;
                for (double value : exp.getDoubleTupleValues().get()){
                    if (value < 0){
                        isNonNegative = false;
                    }
                }
                return isNonNegative;
            }
            return false;
        }
        @Override
        public String msgString() {
            return "a non-negative number";
        }
    },
    BETWEEN_ZERO_AND_ONE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getDoubleValue().isPresent()){
                return exp.getDoubleValue().get() >= 0 && exp.getDoubleValue().get() <= 1;
            }
            else if (exp.getDoubleTupleValues().isPresent()){
                boolean isBetween0And1 = true;
                for (double value : exp.getDoubleTupleValues().get()){
                    if (value < 0 || value > 1){
                        isBetween0And1 = false;
                    }
                }
                return isBetween0And1;
            }
            return false;
        }
        @Override
        public String msgString() {
            return "between one and zero";
        }
    },
    PADDING_TYPE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.PADDING_VALID)
                        || optString.get().equals(AllPredefinedLayers.PADDING_SAME)
                        || optString.get().equals(AllPredefinedLayers.PADDING_NO_LOSS)){
                    return true;
                }
            }
            return false;
        }
        @Override
        protected String msgString() {
            return AllPredefinedLayers.PADDING_VALID + ", "
                    + AllPredefinedLayers.PADDING_SAME + ",  "
                    + AllPredefinedLayers.PADDING_NO_LOSS;
        }
    },
    TRANSPADDING_TYPE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.PADDING_VALID)
                        || optString.get().equals(AllPredefinedLayers.PADDING_SAME)) {
                    return true;
                }
            }
            return false;
        }
        @Override
        protected String msgString() {
            return AllPredefinedLayers.PADDING_VALID + ", "
                    + AllPredefinedLayers.PADDING_SAME;
        }
    },
    PADDING_TYPE3D {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.PADDING_VALID)
                        || optString.get().equals(AllPredefinedLayers.PADDING_VALID3D)
                        || optString.get().equals(AllPredefinedLayers.PADDING_SAME3D)
                        || optString.get().equals(AllPredefinedLayers.PADDING_SIMPLE3D)){
                    return true;
                }
            }
            return false || exp.isIntTuple().get();
        }
        @Override
        protected String msgString() {
            return AllPredefinedLayers.PADDING_VALID3D + ", "
                    + AllPredefinedLayers.PADDING_SAME3D + ",  "
                    + AllPredefinedLayers.PADDING_NO_LOSS + " or "
                    + AllPredefinedLayers.PADDING_SIMPLE3D;
        }
    },
    TRANSPADDING_TYPE3D {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.PADDING_VALID3D)
                        || optString.get().equals(AllPredefinedLayers.PADDING_SAME3D)
                        || optString.get().equals(AllPredefinedLayers.PADDING_SIMPLE3D)){
                    return true;
                }
            }
            return false || exp.isIntTuple().get();
        }
        @Override
        protected String msgString() {
            return AllPredefinedLayers.PADDING_VALID3D + ", "
                    + AllPredefinedLayers.PADDING_SAME3D + " or "
                    + AllPredefinedLayers.PADDING_SIMPLE3D;
        }
    },
    POOL_TYPE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.POOL_MAX)
                        || optString.get().equals(AllPredefinedLayers.POOL_AVG)){
                    return true;
                }
            }
            return false;
        }

        @Override
        protected String msgString() {
            return AllPredefinedLayers.POOL_MAX + " or "
                    + AllPredefinedLayers.POOL_AVG;
        }
    },
    ACTIVATION_TYPE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.MEMORY_ACTIVATION_LINEAR)
                        || optString.get().equals(AllPredefinedLayers.MEMORY_ACTIVATION_RELU)
                        || optString.get().equals(AllPredefinedLayers.MEMORY_ACTIVATION_TANH)
                        || optString.get().equals(AllPredefinedLayers.MEMORY_ACTIVATION_SIGMOID)
                        || optString.get().equals(AllPredefinedLayers.MEMORY_ACTIVATION_SOFTRELU)
                        || optString.get().equals(AllPredefinedLayers.MEMORY_ACTIVATION_SOFTSIGN)){
                    return true;
                }
            }
            return false;
        }
        @Override
        protected String msgString() {
            return AllPredefinedLayers.MEMORY_ACTIVATION_LINEAR + " or "
                    + AllPredefinedLayers.MEMORY_ACTIVATION_RELU + " or "
                    + AllPredefinedLayers.MEMORY_ACTIVATION_TANH + " or "
                    + AllPredefinedLayers.MEMORY_ACTIVATION_SIGMOID + " or "
                    + AllPredefinedLayers.MEMORY_ACTIVATION_SOFTRELU + " or "
                    + AllPredefinedLayers.MEMORY_ACTIVATION_SOFTSIGN;
        }
    },
    MEMORY_REPLACEMENT_STRATEGY_TYPE {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if (optString.get().equals(AllPredefinedLayers.REPLACE_OLDEST)
                        || optString.get().equals(AllPredefinedLayers.NO_REPLACEMENT)){
                    return true;
                }
            }
            return false;
        }

        @Override
        protected String msgString() {
            return AllPredefinedLayers.REPLACE_OLDEST + " or "
                    + AllPredefinedLayers.NO_REPLACEMENT;
        }
    },
    NULLABLE_AXIS {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getIntValue().isPresent()){
                int intValue = exp.getIntValue().get();
                return intValue >= -1 && intValue <= 2; // -1 is null
            }

            return false;
        }

        @Override
        public String msgString() {
            return "an axis between 0 and 2 or -1";
        }
    },
    NULLABLE_AXIS_WITHOUT_2 {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getIntValue().isPresent()){
                int intValue = exp.getIntValue().get();
                return intValue >= -1 && intValue < 2; // -1 is null
            }

            return false;
        }

        @Override
        public String msgString() {
            return "an axis between 0 and 1 or -1";
        }
    },
    AXIS {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getIntValue().isPresent()){
                int intValue = exp.getIntValue().get();
                return intValue >= 0 && intValue <= 2;
            }

            return false;
        }

        @Override
        public String msgString() {
            return "an axis between 0 and 2";
        }
    },
    AXIS_WITHOUT_2 {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            if (exp.getIntValue().isPresent()){
                int intValue = exp.getIntValue().get();
                return intValue >= 0 && intValue <= 1;
            }

            return false;
        }

        @Override
        public String msgString() {
            return "an axis between 0 and 1";
        }
    },
    REPARAMETERIZE_PDFS {
        @Override
        public boolean isValid(ArchSimpleExpressionSymbol exp) {
            Optional<String> optString= exp.getStringValue();
            if (optString.isPresent()){
                if ( optString.get().equals(AllPredefinedLayers.PDF_NORMAL)){
                        //|| optString.get().equals(AllPredefinedLayers.PDF_DIRICHLET)){
                    return true;
                }
            }
            return false;
        }
        @Override
        protected String msgString() {
            return AllPredefinedLayers.PDF_NORMAL; //+ " or "
                    //+ AllPredefinedLayers.PDF_DIRICHLET;

        }
    };

    protected abstract boolean isValid(ArchSimpleExpressionSymbol exp);

    abstract protected String msgString();

    public static boolean check(ParameterSymbol parameter){
        boolean valid = true;
        for (Constraints constraint : parameter.getConstraints()) {
            valid = valid &&
                    constraint.check(parameter.getExpression(), parameter.getSourcePosition(), parameter.getName());
        }
        return valid;
    }

    public static boolean check(ArgumentSymbol argument){
        boolean valid = true;
        ParameterSymbol parameter = argument.getParameter();
        for (Constraints constraint : parameter.getConstraints()) {
            valid = valid &&
                    constraint.check(argument.getRhs(), argument.getSourcePosition(), parameter.getName());
        }
        return valid;
    }

    public boolean check(ArchExpressionSymbol exp, SourcePosition sourcePosition, String name){
        if (exp instanceof ArchRangeExpressionSymbol){
            ArchRangeExpressionSymbol range = (ArchRangeExpressionSymbol)exp;
            if (!INTEGER.check(range.getStartSymbol(), sourcePosition, name)
                    || !INTEGER.check(range.getEndSymbol(), sourcePosition, name)){
                return false;
            }
        }
        for (List<ArchSimpleExpressionSymbol> expList : exp.getElements().get()) {
            for (ArchSimpleExpressionSymbol singleExp : expList) {
                if (!isValid(singleExp)) {
                    Log.error("0" + ILLEGAL_ASSIGNMENT + " Illegal assignment of '" + name + "'. " +
                                    "Expression must be " + msgString() + "."
                            , sourcePosition);
                    return false;
                }
            }
        }
        return true;
    }
}
