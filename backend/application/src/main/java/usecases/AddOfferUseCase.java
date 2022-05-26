package usecases;

import commands.AddOfferCommand;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;

import java.util.UUID;


@AllArgsConstructor
public class AddOfferUseCase implements CommandHandler<AddOfferCommand> {

	// private final DataRowPersistencePort dataRowPersistencePort;
	private final OfferPersistencePort offerPersistencePort;

	public void handle(AddOfferCommand command) {
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
