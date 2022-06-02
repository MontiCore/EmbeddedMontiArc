package ports;

import entity.Offer;

public interface DsManagementPort {

	void deployPolicy(Offer offer);

	void addExecutionPort(DsExecutionPort dsExecutionPort);
}
