package ports;

import entity.Offer;

public interface PolicyManagementPort {

	void deployPolicy(Offer offer);

	void addPxp(DsExecutionPort dsExecutionPort);
}
