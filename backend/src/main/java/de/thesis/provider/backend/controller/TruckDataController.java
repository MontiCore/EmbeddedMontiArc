package de.thesis.provider.backend.controller;

import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.csv.DataRow;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;
import java.util.List;

@RestController
@RequestMapping("truckdata")
@CrossOrigin(origins = "*")
public class TruckDataController {

	@GetMapping
	public List<DataRow> getTruckData() throws IOException {
		CsvReader reader = new CsvReader();
		return reader.getCsvData();
	}
}
