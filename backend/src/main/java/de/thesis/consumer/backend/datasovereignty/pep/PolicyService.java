package de.thesis.consumer.backend.datasovereignty.pep;

import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.service.IPolicyService;

public class PolicyService implements IPolicyService {
	@Override
	public boolean isValid(Policy policy) {
		return true;
	}
}
