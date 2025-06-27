/* (c) https://github.com/MontiCore/monticore */
import Environment;
import ReplayMemory;

schema Reinforcement extends General {

    actor_optimizer: optimizer_type
    environment: environment_type!
    replay_memory = buffer: replay_memory_type
}
