package de.thesis.consumer.backend.presentation.controller;

import de.thesis.consumer.backend.domain.service.OfferService;
import de.thesis.consumer.backend.domain.exception.InvalidPolicyException;
import de.thesis.consumer.backend.persistence.entity.Offer;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;

import java.util.List;
import java.util.UUID;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class OfferController {

	private OfferService service;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void offer(@RequestBody Offer offer) {
		try {
			service.offerDataset(offer);
		} catch (InvalidPolicyException e) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Dataset policy is invalid");
		}
	}

	@GetMapping
	public List<Offer> getAllOffers() {
		return service.getAllOffers();
	}

	@PostMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void buyOffer(@PathVariable UUID offerId) {
		service.buyOffer(offerId);
	}
}
