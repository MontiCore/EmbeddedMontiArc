package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.domain.model.Policy;

public interface IPolicyManagementPoint {

	void instantiatePolicy(Policy policy) throws PolicyInstantiationException;
}
