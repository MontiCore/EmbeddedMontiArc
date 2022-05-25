package presentation.controllers;

import commands.AddOfferCommand;
import exceptions.UseCaseException;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;
import usecases.AddOfferUseCase;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin(origins = "*")
@Slf4j
public class OfferController {

	private AddOfferUseCase addOfferUseCase;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void offer(@RequestBody AddOfferCommand offer) {
		try {
			addOfferUseCase.handle(offer);
		} catch (UseCaseException exception) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Dataset could not be offered", exception);
		}
	}
}
