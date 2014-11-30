#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include <vector>
#include <ras_utils/genetic/individual.h>
#include <ras_utils/graph/graph.h>
#include <sys/time.h>
#include <ctime>
#include <queue>

// ** CONFIGURATION PARAMETERS
#define N_GENERATIONS 5000      // Number of generations
#define N_INDIVIDUALS 100       // Number of individuals per generation
#define N_ELLITE      2         // Number of ellite individuals
#define P_CROSSOVER   0.7       // Cross-over probability
#define P_MUTATION    0.005     // Mutation probability


struct Generation
{
    std::vector<Individual> population_;
    double total_fitness_;
    std::vector<double> cum_distribution_;
};

class GeneticAlgorithm
{
public:
    GeneticAlgorithm(const Graph &graph);

    void computeSolution(std::vector<int> &path);
    void computeSolution(std::vector<Node> &path);

    Individual &getBestIndividual();

private:
    Graph graph_;
    int n_nodes_;
    Generation currentGeneration_;
    Individual bestIndividual_;
    double bestFitness_;

    // Help variables
    std::vector<int> possibleNodes_;

    void initializeGeneration();
        Individual createRandomIndividual();

    void evolveGeneration();
        void evaluation();
        void ellite_selection(Generation& newGeneration);
        void selection(Generation& newGeneration);
        void crossover(Generation &newGeneration);
            void coupleCrossover(Individual &parent1, Individual &parent2,
                                 Individual &child1, Individual &child2);
        void mutation(Generation& newGeneration);
        void evaluation2(Generation &newGeneration);

};

#endif // GENETIC_ALGORITHM_H
