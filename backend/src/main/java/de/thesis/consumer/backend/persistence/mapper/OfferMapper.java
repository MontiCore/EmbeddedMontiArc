package de.thesis.consumer.backend.persistence.mapper;

import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import org.springframework.stereotype.Component;

@Component
public class OfferMapper implements Mapper<Offer, OfferEntity>{

	@Override
	public OfferEntity mapTo(Offer offer) {
		OfferEntity entity = new OfferEntity();
		entity.setId(offer.getId());
		entity.setTitle(offer.getTitle());
		entity.setDescription(offer.getDescription());
		entity.setPrice(offer.getPrice());
		entity.setProvider(offer.getProvider());

		return entity;
	}

	@Override
	public Offer mapFrom(OfferEntity entity) {
		Offer offer = new Offer();
		offer.setId(entity.getId());
		offer.setTitle(entity.getTitle());
		offer.setDescription(entity.getDescription());
		offer.setPrice(entity.getPrice());
		offer.setProvider(entity.getProvider());

		return offer;
	}
}
