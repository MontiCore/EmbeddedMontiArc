package de.thesis.consumer.backend.domain;

import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import de.thesis.consumer.backend.domain.model.Policy;

import java.io.IOException;

public interface IPolicyManagementPoint {

	void instantiatePolicy(Policy policy) throws PolicyInstantiationException, ConflictingResourceException, IOException, NoSuchEntityException, InvalidEntityException, ResourceUpdateException;
}
