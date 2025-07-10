package datasovereignty;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import entity.Policy;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.PolicyDeploymentPort;

@Component
@AllArgsConstructor
@Slf4j
public class MydataPolicyDeploymentPortAdapter implements PolicyDeploymentPort {

	private final MydataPolicyFactory policyFactory;
	private final IMyDataEnvironment myDataEnvironment;

	@Override
	public void deployPolicy(Policy policy) {
		try {
			String rawMyDataPolicy = policyFactory.getMydataPolicy(policy);
			myDataEnvironment.getPmp().deployPolicy(
					myDataEnvironment.getPmp().addPolicy(
							new de.fraunhofer.iese.mydata.policy.Policy(
									rawMyDataPolicy
							)
					));

			log.info("Policy deployed successfully:");
			log.info(rawMyDataPolicy);
		} catch (Exception e) {
			throw new RuntimeException(String.format("Policy with target %s could not be deployed", policy.getTargetId()));
		}
	}
}
