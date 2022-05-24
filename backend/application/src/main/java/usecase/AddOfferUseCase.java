package usecase;

import dto.AddOfferCommand;
import entity.Offer;
import exception.PolicyInvalidException;
import lombok.AllArgsConstructor;
import port.OfferPersistencePort;
import port.PolicyManagementPort;

import java.util.UUID;

@AllArgsConstructor
public class AddOfferUseCase {

	private final OfferPersistencePort offerPersistencePort;
	private final PolicyManagementPort policyValidationPort;

	public void addOffer(AddOfferCommand command) throws PolicyInvalidException {
		if (!policyValidationPort.isValid(command.getPolicy())) {
			throw new PolicyInvalidException("Policy invalid");
		}

		Offer offer = new Offer(
				UUID.randomUUID(),
				command.getTitle(),
				command.getProvider(),
				command.getDescription(),
				command.getPrice(),
				command.getPolicy(),
				command.getExpiresOn(),
				command.getData(),
				command.getLoggingUrl()
		);

		offerPersistencePort.save(offer);
	}
}
