package datasovereingty;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import entity.Policy;
import entity.Timer;
import lombok.AllArgsConstructor;
import ports.PolicyManagementPort;

@AllArgsConstructor
public class PolicyManagementPointAdapter implements PolicyManagementPort {

	private final IMyDataEnvironment myDataEnvironment;
	// TODO die Policies und Timer richtig ins Domain model aufnehmen
	@Override
	public void deployPolicy(Policy policy) {
		try {
			myDataEnvironment.getPmp().deployPolicy(
					myDataEnvironment.getPmp().addPolicy(
							new de.fraunhofer.iese.mydata.policy.Policy(policy.getRawValue())));
		} catch (Exception e) {
			throw new RuntimeException(String.format("Policy %s could not be deployed", policy.getId()));
		}
	}

	@Override
	public void deployTimer(Timer timer) {
		try {
			myDataEnvironment.getPmp().deployTimer(myDataEnvironment.getPmp().addTimer(new de.fraunhofer.iese.mydata.timer.Timer(timer.getRawValue())));
		} catch (Exception e) {
			e.printStackTrace();
		}

	}
}
