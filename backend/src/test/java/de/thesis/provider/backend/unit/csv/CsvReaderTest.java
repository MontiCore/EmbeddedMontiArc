package de.thesis.provider.backend.unit.csv;

import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.csv.DataRow;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.mock.web.MockMultipartFile;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;

import static org.assertj.core.api.AssertionsForClassTypes.assertThat;

class CsvReaderTest {

	@Test
	void shouldReturnFourDataRows() throws IOException {
		MockMultipartFile file
				= new MockMultipartFile(
				"file",
				"hello.txt",
				MediaType.TEXT_PLAIN_VALUE,
				("id;latitude;longitude;gpsTime;speed;heading;totalFuelUsed;dayId;timestamp;odometer\n" +
						"1611482441;50.672425;3.234842;2021-08-18T07:36:27Z;0;13;0;71919bfcd628df321666fc97aac1c23b501d195b096e38bdcc5b7ce98b9bdd34;2021-08-18T07:36:36Z;358850").getBytes()
		);

		CsvReader reader = new CsvReader();
		List<DataRow> data = reader.getCsvData(file);

		assertThat(data.size()).isEqualTo(1);
		assertThat(data.get(0).getId()).isEqualTo(1611482441);
		assertThat(data.get(0).getLatitude()).isEqualTo(50.672425);
		assertThat(data.get(0).getLongitude()).isEqualTo(3.234842);
		assertThat(data.get(0).getGpsTime()).isEqualTo(LocalDateTime.of(2021, 8, 18, 7, 36, 27));
		assertThat(data.get(0).getSpeed()).isEqualTo(0);
		assertThat(data.get(0).getHeading()).isEqualTo(13);
		assertThat(data.get(0).getTotalFuelUsed()).isEqualTo(0);
		assertThat(data.get(0).getDayID()).isEqualTo("71919bfcd628df321666fc97aac1c23b501d195b096e38bdcc5b7ce98b9bdd34");
		assertThat(data.get(0).getTimestamp()).isEqualTo(LocalDateTime.of(2021, 8, 18, 7, 36, 36));
		assertThat(data.get(0).getOdometer()).isEqualTo(358850);
	}
}
