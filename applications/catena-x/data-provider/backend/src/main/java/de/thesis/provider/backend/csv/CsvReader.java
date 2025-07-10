package de.thesis.provider.backend.csv;

import com.opencsv.bean.CsvToBean;
import com.opencsv.bean.CsvToBeanBuilder;
import org.springframework.web.multipart.MultipartFile;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.util.List;

public class CsvReader {
	public List<DataRow> getCsvData(MultipartFile file) throws IOException {
		Reader reader = new BufferedReader(new
				InputStreamReader(file.getInputStream(), StandardCharsets.UTF_8));
		CsvToBean<DataRow> csvReader = new CsvToBeanBuilder<DataRow>(reader)
				.withType(DataRow.class)
				.withSeparator(';')
				.withIgnoreLeadingWhiteSpace(true)
				.withIgnoreEmptyLine(true)
				.build();

		return csvReader.parse();
	}
}
