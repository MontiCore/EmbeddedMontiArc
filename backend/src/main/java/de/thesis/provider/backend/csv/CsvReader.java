package de.thesis.provider.backend.csv;

import com.opencsv.bean.CsvToBean;
import com.opencsv.bean.CsvToBeanBuilder;
import org.springframework.core.io.ClassPathResource;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.List;

public class CsvReader {
	public List<DataRow> getCsvData() throws IOException {
		Reader reader = new BufferedReader(new FileReader(new ClassPathResource(
				"csv/bigtest.csv").getFile()));
		CsvToBean<DataRow> csvReader = new CsvToBeanBuilder<DataRow>(reader)
				.withType(DataRow.class)
				.withSeparator(';')
				.withIgnoreLeadingWhiteSpace(true)
				.withIgnoreEmptyLine(true)
				.build();

		return csvReader.parse();
	}
}
