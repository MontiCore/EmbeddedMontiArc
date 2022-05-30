package usecases;

import commands.AddOfferCommand;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;

import java.util.UUID;


@AllArgsConstructor
public class AddOfferUseCase implements CommandHandler<AddOfferCommand, Offer> {
	private final OfferPersistencePort offerPersistencePort;
	public Offer handle(AddOfferCommand command) {
		Offer offer = new Offer(
				UUID.randomUUID(),
				command.getMetadata(),
				command.getData()
		);

		offerPersistencePort.save(offer);

		return offer;
	}
}
