package de.thesis.consumer.backend.datasovereignty.pep;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.PolicyInstantiationException;
import de.thesis.consumer.backend.domain.model.Policy;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.io.IOException;

@Component
@AllArgsConstructor
public class PolicyManagementPoint implements IPolicyManagementPoint {

	private final IMyDataEnvironment myDataEnv;

	@Override
	public void instantiatePolicy(Policy policy) throws PolicyInstantiationException {
		try {
			myDataEnv.getPmp().deployPolicy(myDataEnv.getPmp().addPolicy(new de.fraunhofer.iese.mydata.policy.Policy(
					policy.getRawValue()
			)));
		} catch (IOException | NoSuchEntityException | ResourceUpdateException | InvalidEntityException | ConflictingResourceException e) {
			throw new PolicyInstantiationException(String.format("Error instantiating policy with ID %s", policy.getId()));
		}
	}
}
