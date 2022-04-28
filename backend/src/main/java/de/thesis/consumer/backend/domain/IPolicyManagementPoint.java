package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.model.Timer;

public interface IPolicyManagementPoint {

	void deployPolicy(Policy policy);

	void deployTimer(Timer timer);
}
