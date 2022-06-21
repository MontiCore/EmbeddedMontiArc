package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.Id;
import javax.persistence.Table;
import java.time.LocalDate;
import java.time.LocalTime;
import java.util.UUID;

@Entity
@Table(name = "policy")
@Getter
@Setter
public class PolicyEntity {
	@Id
	@GeneratedValue
	private int id;
	private UUID targetId;
	private LocalTime startTime;
	private LocalTime endTime;
	private LocalDate expiresOn;
	private Integer maxUsages;
	private boolean localLogging;
	private boolean remoteLogging;
}

