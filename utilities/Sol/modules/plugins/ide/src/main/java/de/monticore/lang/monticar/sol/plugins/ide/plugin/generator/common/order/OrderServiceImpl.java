/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.order;

import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.TaskSymbol;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DirectedAcyclicGraph;
import org.jgrapht.traverse.BreadthFirstIterator;

import java.util.*;
import java.util.stream.Collectors;

@Singleton
public class OrderServiceImpl implements OrderService {
    @Override
    public List<List<TaskSymbol>> orderTasks(ConfigurationTypeSymbol configuration) {
        return this.orderTasks(this.buildTaskGraph(configuration));
    }

    @Override
    public List<List<ConfigurationSymbol>> orderConfigurations(List<ConfigurationSymbol> configurations) {
        List<List<ConfigurationSymbol>> result = new ArrayList<>();
        Map<Integer, List<ConfigurationSymbol>> groups = configurations.stream()
                .collect(Collectors.groupingBy(configuration -> configuration.getOrder().orElse(0)));
        List<Integer> positions = groups.keySet().stream()
                .sorted()
                .collect(Collectors.toList());

        positions.forEach(position -> result.add(groups.get(position)));

        return result;
    }

    protected Graph<TaskSymbol, DefaultEdge> buildTaskGraph(ConfigurationTypeSymbol configuration) {
        Graph<TaskSymbol, DefaultEdge> result = new DirectedAcyclicGraph<>(DefaultEdge.class);
        List<TaskSymbol> tasks = configuration.getTaskSymbols();

        tasks.forEach(task -> {
            result.addVertex(task);
            task.getPredecessorSymbols().forEach(predecessor -> {
                result.addVertex(predecessor);
                result.addEdge(predecessor, task);
            });
        });

        return result;
    }

    protected List<List<TaskSymbol>> orderTasks(Graph<TaskSymbol, DefaultEdge> graph) {
        Set<TaskSymbol> startTasks = graph.vertexSet().stream()
                .filter(task -> task.getPredecessorSymbols().isEmpty())
                .collect(Collectors.toSet());

        return this.doOrderTasks(graph, startTasks);
    }

    protected List<List<TaskSymbol>> doOrderTasks(Graph<TaskSymbol, DefaultEdge> graph, Set<TaskSymbol> startTasks) {
        List<List<TaskSymbol>> result = new ArrayList<>();
        BreadthFirstIterator<TaskSymbol, DefaultEdge> iterator = new BreadthFirstIterator<>(graph, startTasks);

        while (iterator.hasNext()) {
            TaskSymbol task = iterator.next();
            int depth = iterator.getDepth(task);

            if (result.size() < depth + 1) result.add(new ArrayList<>());

            result.get(depth).add(task);
        }

        return result;
    }
}
