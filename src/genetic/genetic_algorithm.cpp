#include <ras_utils/genetic/genetic_algorithm.h>

GeneticAlgorithm::GeneticAlgorithm(const Graph& graph)
{
    // ** Set random seed
    srand(std::time(NULL));

    graph_ = graph;
    n_nodes_ = graph.getNodeCount();

    //Help variable to initialize individuals
    for (unsigned int i = 0; i < n_nodes_; i++)
        possibleNodes_.push_back(i);
}

void GeneticAlgorithm::computeSolution(std::vector<Node> &path)
{
    std::vector<int> path_idx;
    this->computeSolution(path_idx);
    // Transform from indexes to actual nodes
    for(std::size_t i = 0; i < path_idx.size(); ++i)
    {
        const Node &n = this->graph_.getNodes()[path_idx[i]];
        path.push_back(n);
    }
}

void GeneticAlgorithm::computeSolution(std::vector<int> &path)
{
    // ** Initialize population
    initializeGeneration();

    // ** Evolve through the generations
    for(unsigned int i = 0; i < N_GENERATIONS; ++i)
    {
        struct timeval t1, t2;
        gettimeofday(&t1, NULL);

        evolveGeneration();

        gettimeofday(&t2, NULL);
        std::cout << "[Generation "<<i<<"] Fitness: "<<currentGeneration_.total_fitness_
                                    <<" Best fitness: "<< bestFitness_
                  << " ["<< (t2.tv_usec - t1.tv_usec)/1000.0 << " ms]"<<std::endl;
    }
    // ** Get best individual
    Individual &best = this->getBestIndividual();
    path = best.getGenes();
}

Individual& GeneticAlgorithm::getBestIndividual(){return bestIndividual_;}

void GeneticAlgorithm::initializeGeneration()
{
    // ** Generate random individuals
    currentGeneration_.population_.clear();
    currentGeneration_.cum_distribution_.clear();

    for(unsigned int i = 0; i < N_INDIVIDUALS; i++)
    {
        Individual ind = createRandomIndividual();
        currentGeneration_.population_.push_back(ind);
        currentGeneration_.total_fitness_ += ind.getFitness();
        currentGeneration_.cum_distribution_.push_back(0.0);
    }
}

Individual GeneticAlgorithm::createRandomIndividual()
{
    std::vector<int> path(possibleNodes_);
    std::random_shuffle(path.begin()+1, path.end()-1); //The origin and end is always the (0,0) point
    std::cout << "First: "<<path[0] <<"; Last: "<<path[path.size()-1]<<std::endl;
    double cost = graph_.computePathCost(path);
    Individual individual(path, 1.0/cost);
    individual.print();
    return individual;
}

void GeneticAlgorithm::evolveGeneration()
{
    Generation newGeneration;
    newGeneration.population_.clear();
    newGeneration.total_fitness_ = 0.0;
    newGeneration.cum_distribution_ = std::vector<double>(currentGeneration_.cum_distribution_.size());
    // ** Evaluation (In order to compute survival probabilities)
    evaluation();

    // ** Ellite selection
    if(N_ELLITE > 0)
        ellite_selection(newGeneration);

    // ** Selection
    selection(newGeneration);

    // ** Cross-over
    crossover(newGeneration);

    // ** Mutation
    mutation(newGeneration);

    // ** Evaluation2 (In order to select the best individual)
    evaluation2(newGeneration);

    // ** Update current population
    currentGeneration_ = newGeneration;
}

void GeneticAlgorithm::evaluation()
{
    // ** Sort according to fitness
    std::sort(currentGeneration_.population_.begin(),
              currentGeneration_.population_.end());

    // ** Compute the survival probability for each individual and cumulative distribution
    for(unsigned int i = 0; i < currentGeneration_.population_.size(); i++)
    {
        // Probability
        Individual& ind = currentGeneration_.population_.at(i);
        ind.setProbability(ind.getFitness()/currentGeneration_.total_fitness_);

        // Cumulative distribution
        if(i == 0)
            currentGeneration_.cum_distribution_.at(i) = ind.getProbability();
        else
            currentGeneration_.cum_distribution_.at(i) =
                    currentGeneration_.cum_distribution_.at(i-1) + ind.getProbability();
    }
}

void GeneticAlgorithm::evaluation2(Generation& newGeneration)
{
    for(unsigned int i = 0; i < newGeneration.population_.size(); i++)
    {
        Individual& ind = newGeneration.population_.at(i);

        double fitness =  1.0/graph_.computePathCost(ind.getGenes());
        ind.setFitness(fitness);
        newGeneration.total_fitness_ += fitness;

        // ** Update best individual
        if (fitness > bestFitness_)
        {
            bestIndividual_ = newGeneration.population_.at(i);
            bestFitness_ = fitness;
        }
    }
}

void GeneticAlgorithm::ellite_selection(Generation& newGeneration)
{
    for(unsigned int i = currentGeneration_.population_.size() -1;
                     i > currentGeneration_.population_.size() -1 - N_ELLITE; i--)
    {
        newGeneration.population_.push_back(currentGeneration_.population_.at(i));
    }
}

void GeneticAlgorithm::selection(Generation &newGeneration)
{
    // ** Multinomial resampling
    while(newGeneration.population_.size() < N_INDIVIDUALS)
    {
        double r = ((double) rand() / (RAND_MAX));
        for (unsigned int i = 0; i < currentGeneration_.cum_distribution_.size(); i++)
        {
            if (currentGeneration_.cum_distribution_.at(i) >= r)
            {
                newGeneration.population_.push_back(currentGeneration_.population_.at(i));
                break;
            }
        }
    }
}

void GeneticAlgorithm::crossover(Generation& newGeneration)
{
    Generation tmpGeneration;
    tmpGeneration.population_.clear();

    // ** Create crossovers from original population
    for(unsigned int i = N_ELLITE; i < newGeneration.population_.size(); i += 2)
    {
        // Parents
        int p1 = rand() % (newGeneration.population_.size() - N_ELLITE) + N_ELLITE;
        int p2 = rand() % (newGeneration.population_.size() - N_ELLITE) + N_ELLITE;
        Individual child1, child2;

        // Cross-over with probability P_CROSSOVER
        double r = ((double) rand() / (RAND_MAX));

        if( r <= P_CROSSOVER) // Cross-over
        {
            Individual parent1 = newGeneration.population_.at(p1);
            Individual parent2 = newGeneration.population_.at(p2);

            coupleCrossover(parent1, parent2, child1, child2);
        }
        else // The parents go to the next generation
        {
            child1 = newGeneration.population_.at(p1);
            child2 = newGeneration.population_.at(p2);
        }
        tmpGeneration.population_.push_back(child1);
        tmpGeneration.population_.push_back(child2);
    }

    // ** Update population of the new generation
    for(unsigned int i =  N_ELLITE, j = 0; j < tmpGeneration.population_.size(); ++i, ++j)
    {
        newGeneration.population_.at(i) = tmpGeneration.population_.at(j);
    }
}

void GeneticAlgorithm::coupleCrossover(Individual& parent1, Individual& parent2,
                                       Individual& child1, Individual& child2)
{
    child1.getGenes().clear();
    child2.getGenes().clear();

    // ** Create random cross-over point
    int r = rand() % (parent1.getGenes().size());

    // ** Copy first part from parents to childs
    for (unsigned int i = 0; i < r; i++)
    {
        int gen1 = parent1.getGenes().at(i);
        int gen2 = parent2.getGenes().at(i);

        child1.getGenes().push_back(gen1);
        child2.getGenes().push_back(gen2);
    }

    // ** Copy second part from the opposite parent
    for (unsigned int i = 0; i < parent1.getGenes().size(); i++)
    {
        int gen1 = parent1.getGenes().at(i);
        int gen2 = parent2.getGenes().at(i);

        if(!child1.containsGene(gen2))
            child1.getGenes().push_back(gen2);

        if(!child2.containsGene(gen1))
            child2.getGenes().push_back(gen1);
    }
}

void GeneticAlgorithm::mutation(Generation &newGeneration)
{
    for(unsigned int i = 0; i < newGeneration.population_.size(); i++)
    {
        // Mutate with probability P_MUTATION
        double r = ((double) rand() / (RAND_MAX));

        if(r <= P_MUTATION)
            newGeneration.population_.at(i).mutate();
    }
}
