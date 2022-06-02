package datasovereignty;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import entity.Offer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.DsExecutionPort;
import ports.DsManagementPort;

@Component
@AllArgsConstructor
@Slf4j
public class MydataDsManagementPointAdapter implements DsManagementPort {

	private final MydataPolicyFactory policyFactory;
	private final IMyDataEnvironment myDataEnvironment;

	@Override
	public void deployPolicy(Offer offer) {
		try {
			System.err.println(policyFactory.getMydataPolicy(offer));
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
	public void addExecutionPort(DsExecutionPort dsExecutionPort) {
		try {
			myDataEnvironment.registerLocalPxp(dsExecutionPort.getName(), dsExecutionPort);
		} catch (Exception e) {
			log.info("Policy execution point {} could not be deployed", dsExecutionPort.getName());
			throw new RuntimeException(e);
		}
	}
}
