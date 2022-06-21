package entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDate;
import java.time.LocalTime;
import java.util.UUID;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class Policy {
	private int id;
	private UUID targetId;
	private LocalTime startTime;
	private LocalTime endTime;
	private LocalDate expiresOn;
	private Integer maxUsages;
	private boolean localLogging;
	private boolean remoteLogging;
}
