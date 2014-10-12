#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H

#include <vector>
#include <iostream>
#include <cstdlib>

class Individual
{
public:
    Individual();
    Individual(std::vector<int> genes, double fitness);

    std::vector<int>& getGenes();
    double getFitness() const;
    void setFitness(double fitness);

    void print() const ;

    bool operator< (const Individual& src) const;

    void setProbability(double probability);
    double getProbability() const;

    void mutate();
    bool containsGene(int g);

private:
    std::vector<int> genes_;
    double fitness_;
    double probability_;
};

#endif // INDIVIDUAL_H
