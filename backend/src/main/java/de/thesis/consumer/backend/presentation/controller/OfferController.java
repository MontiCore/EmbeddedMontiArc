package de.thesis.consumer.backend.presentation.controller;

import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import de.thesis.consumer.backend.domain.PolicyInstantiationException;
import de.thesis.consumer.backend.domain.exception.InvalidPolicyException;
import de.thesis.consumer.backend.domain.exception.PolicyNotFoundException;
import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.service.OfferService;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;

import java.io.IOException;
import java.util.List;
import java.util.UUID;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin(origins = "*")
@Slf4j
public class OfferController {

	private OfferService service;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void offer(@RequestBody Offer offer) {
		try {
			service.offer(offer);
		} catch (PolicyNotFoundException e) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Policy not found");
		} catch (Exception e) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Dataset policy is invalid");
		}
	}

	@GetMapping
	public List<Offer> getAllOffers() {
		return service.getAllOffers();
	}

	@PostMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void buyOffer(@PathVariable UUID offerId) throws PolicyInstantiationException, ConflictingResourceException, IOException, NoSuchEntityException, InvalidEntityException, ResourceUpdateException, PolicyNotFoundException {
		service.buyOffer(offerId);
	}
}
