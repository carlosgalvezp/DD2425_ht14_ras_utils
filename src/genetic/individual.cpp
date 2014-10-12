#include <ras_utils/genetic/individual.h>

Individual::Individual()
{
}

Individual::Individual(std::vector<int> genes, double fitness)
{
    genes_ = genes;
    fitness_ = fitness;
}

void Individual::print() const
{
    for(unsigned int i = 0; i < genes_.size(); ++i)
    {
        std::cout << genes_.at(i) ;
        if(i < genes_.size() - 1)
            std::cout << "->";
        else
            std::cout << " [" << fitness_<<"]" << std::endl;
    }
}

bool Individual::operator< (const Individual& src) const
{
    return (fitness_ > src.fitness_);
}

void Individual::setProbability(double probability) {probability_ = probability;}
void Individual::setFitness(double fitness) {fitness_= fitness;}

double Individual::getProbability()                 const {return probability_;}
double Individual::getFitness()                     const {return fitness_; }
std::vector<int>& Individual::getGenes()                  {return genes_;}

void Individual::mutate()
{
    bool finished(false);
    while (!finished)
    {
        // Generate two random positions between 1 and genes.size() - 1
        int p1 = rand() % (genes_.size() - 1) + 1;
        int p2 = rand() % (genes_.size() - 1) + 1;

        // Swap positions
        if (p1 != p2)
        {
            int tmp = genes_.at(p1);
            genes_.at(p1) = genes_.at(p2);
            genes_.at(p2) = tmp;
            finished = true;
        }
    }
}

bool Individual::containsGene(int g)
{
    for(unsigned int i = 0; i < genes_.size(); i++)
    {
        if(genes_.at(i) == g)
            return true;
    }
    return false;
}
