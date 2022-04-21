package de.thesis.consumer.backend.datasovereignty;

import de.thesis.consumer.backend.domain.InhibitionException;
import de.thesis.consumer.backend.domain.PolicyEnforcementPoint;
import de.thesis.consumer.backend.domain.model.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

@Component
@AllArgsConstructor
public class DatasetPolicyEnforcementPoint implements PolicyEnforcementPoint<Dataset> {
	@Override
	public Dataset enforce(Dataset object) throws InhibitionException {
		return object;
	}
}
