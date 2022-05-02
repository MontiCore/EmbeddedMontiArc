package de.thesis.provider.backend.controller;

import de.thesis.provider.backend.DatasetCreateCommand;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/dataset")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class DatasetController {

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public void createDataset(@RequestBody DatasetCreateCommand command) {
		System.err.println("done");
	}
}
