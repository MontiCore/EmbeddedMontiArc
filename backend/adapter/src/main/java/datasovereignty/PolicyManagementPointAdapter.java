package datasovereignty;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import entity.Offer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.PolicyExecutionPoint;
import ports.PolicyManagementPort;

@Component
@AllArgsConstructor
@Slf4j
public class PolicyManagementPointAdapter implements PolicyManagementPort {

	private final MydataPolicyFactory policyFactory;
	private final IMyDataEnvironment myDataEnvironment;

	@Override
	public void deployPolicy(Offer offer) {
		try {
			myDataEnvironment.getPmp().deployPolicy(
					myDataEnvironment.getPmp().addPolicy(
							new de.fraunhofer.iese.mydata.policy.Policy(
									policyFactory.getMydataPolicy(offer)
							)
					));
		} catch (Exception e) {
			throw new RuntimeException(String.format("Policy for offer %s could not be deployed", offer.getId()));
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
