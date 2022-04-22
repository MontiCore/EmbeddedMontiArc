package de.thesis.consumer.backend.domain.service;

import de.thesis.consumer.backend.domain.model.Policy;

public interface IPolicyService {

	boolean isValid(Policy policy);
}
