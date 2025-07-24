package schemalang.validation;

import com.google.common.collect.Lists;

import java.util.Collection;
import java.util.Objects;

import static schemalang.ErrorCodes.ERROR_CODE_TA_00C;
import static schemalang.ErrorCodes.ERROR_MSG_TA_00C;

public class ReferenceModelViolation extends Violation {

    private String referenceModel;
    private Collection<Violation> violations;

    public ReferenceModelViolation(String referenceModel) {
        super(ERROR_CODE_TA_00C, String.format(ERROR_MSG_TA_00C, referenceModel));
        setReferenceModel(referenceModel);
        setViolations(Lists.newArrayList());
    }

    public ReferenceModelViolation(String referenceModel, Collection<Violation> violations) {
        super(ERROR_CODE_TA_00C, String.format(ERROR_MSG_TA_00C, referenceModel));
        setReferenceModel(referenceModel);
        setViolations(violations);
    }

    public String getReferenceModel() {
        return referenceModel;
    }

    public void setReferenceModel(String referenceModel) {
        this.referenceModel = referenceModel;
    }

    public Collection<Violation> getViolations() {
        return violations;
    }

    public void setViolations(Collection<Violation> violations) {
        this.violations = violations;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        ReferenceModelViolation that = (ReferenceModelViolation) o;
        return referenceModel.equals(that.referenceModel) && violations.equals(that.violations);
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), referenceModel, violations);
    }

    @Override
    public String toString() {
        return errorCode.concat(errorMessage);
    }
}