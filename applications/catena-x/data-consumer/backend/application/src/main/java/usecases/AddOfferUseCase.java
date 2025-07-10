package usecases;

import commands.AddOfferCommand;
import entity.Offer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.OfferPersistencePort;

import java.util.UUID;


@AllArgsConstructor
@Slf4j
public class AddOfferUseCase implements CommandHandler<AddOfferCommand> {
	private final OfferPersistencePort offerPersistencePort;

	public void handle(AddOfferCommand command) {
		Offer offer = new Offer(
				UUID.randomUUID(),
				command.getMetadata(),
				command.getData()
		);
		offerPersistencePort.save(offer);

		log.info("Created offer {}", offer.getId().toString());
	}
}
