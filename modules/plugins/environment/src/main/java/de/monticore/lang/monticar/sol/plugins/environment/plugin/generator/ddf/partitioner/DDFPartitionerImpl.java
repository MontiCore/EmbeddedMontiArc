/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.partitioner;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;

import java.util.ArrayList;
import java.util.List;

@Singleton
public class DDFPartitionerImpl implements DDFPartitioner {
    protected final NotificationService notifications;
    protected final List<List<ASTInstruction>> partitions;

    @Inject
    protected DDFPartitionerImpl(NotificationService notifications) {
        this.notifications = notifications;
        this.partitions = new ArrayList<>();
    }

    @Override
    public List<List<ASTInstruction>> partition(List<ASTInstruction> instructions) {
        this.partitions.clear();
        this.notifications.info("Partition AST.");
        instructions.forEach(this::addInstructionToPartition);
        return this.partitions;
    }

    protected void addInstructionToPartition(ASTInstruction instruction) {
        String type = instruction.getType();

        if (this.partitions.size() == 0) this.createNewPartitionFor(instruction);
        else if (this.getCurrentPartitionType().equals(type)) this.getCurrentPartition().add(instruction);
        else this.createNewPartitionFor(instruction);
    }

    protected void createNewPartitionFor(ASTInstruction instruction) {
        List<ASTInstruction> partition = new ArrayList<>();

        partition.add(instruction);
        this.partitions.add(partition);
    }

    protected List<ASTInstruction> getCurrentPartition() {
        return this.partitions.get(this.partitions.size() - 1);
    }

    protected String getCurrentPartitionType() {
        List<ASTInstruction> instructions = this.getCurrentPartition();

        return instructions.size() == 0 ? "" : instructions.get(0).getType();
    }
}
