package de.thesis.consumer.backend.persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.Entity;
import javax.persistence.Id;
import javax.persistence.Table;
import java.util.UUID;

@Entity
@Table(name = "policy")
@Getter
@Setter
public class PolicyEntity {

	@Id
	private String id;
	private String rawValue;
}
