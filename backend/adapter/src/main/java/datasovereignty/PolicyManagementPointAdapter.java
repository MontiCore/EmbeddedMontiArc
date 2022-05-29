package datasovereignty;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import entity.Policy;
import entity.Timer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.PolicyExecutionPoint;
import ports.PolicyManagementPort;

import java.io.IOException;

@Component
@AllArgsConstructor
@Slf4j
public class PolicyManagementPointAdapter implements PolicyManagementPort {

	private final IMyDataEnvironment myDataEnvironment;
	// TODO die Policies und Timer richtig ins Domain model aufnehmen
	@Override
	public void deployPolicy(Policy policy) {
//		try {
//			myDataEnvironment.getPmp().deployPolicy(
//					myDataEnvironment.getPmp().addPolicy(
//							new de.fraunhofer.iese.mydata.policy.Policy(policy.getRawValue())));
//		} catch (Exception e) {
//			throw new RuntimeException(String.format("Policy %s could not be deployed", policy.getId()));
//		}
	}

	@Override
	public void deployTimer(Timer timer) {
		try {
			myDataEnvironment.getPmp().deployTimer(myDataEnvironment.getPmp().addTimer(new de.fraunhofer.iese.mydata.timer.Timer(timer.getRawValue())));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void addPxp(PolicyExecutionPoint policyExecutionPoint) {
		try {
			myDataEnvironment.registerLocalPxp(policyExecutionPoint.getName(), policyExecutionPoint);
		} catch (Exception e) {
			log.info("Policy execution point {} could not be deployed", policyExecutionPoint.getName());
			throw new RuntimeException(e);
		}
	}
}
