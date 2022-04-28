package de.thesis.consumer.backend.datasovereignty.pep;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.model.Timer;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

@Component
@AllArgsConstructor
public class PolicyManagementPoint implements IPolicyManagementPoint {

	private final IMyDataEnvironment myDataEnv;

	@Override
	public void deployPolicy(Policy policy) {
		try {
			myDataEnv.getPmp().deployPolicy(
					myDataEnv.getPmp().addPolicy(
							new de.fraunhofer.iese.mydata.policy.Policy(policy.getRawValue())
					)
			);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void deployTimer(Timer timer) {
		try {
			myDataEnv.getPmp().deployTimer(myDataEnv.getPmp().addTimer(new de.fraunhofer.iese.mydata.timer.Timer(timer.getRawValue())));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
