#    This file is part of DEAP.
#
#    DEAP is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation, either version 3 of
#    the License, or (at your option) any later version.
#
#    DEAP is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with DEAP. If not, see <http://www.gnu.org/licenses/>.


#    example which maximizes the sum of a list of integers
#    each of which can be 0 or 1

import random

from deap import base
from deap import creator
from deap import tools

creator.create("FitnessMax", base.Fitness, weights=(1.0,)) 
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()

# Attribute generator 
#                      define 'attr_bool' to be an attribute ('gene')
#                      which corresponds to integers sampled uniformly
#                      from the range [0,1] (i.e. 0 or 1 with equal
#                      probability)
toolbox.register("attr_points", random.randint, 1, 10)
toolbox.register("attr_xmin", random.uniform,0.01,2.0)
toolbox.register("attr_xmax", random.uniform,0.01,2.0)
toolbox.register("attr_ymin", random.uniform,0.01,2.0)
toolbox.register("attr_ymax", random.uniform,0.01,2.0)
toolbox.register("attr_zmin", random.uniform,0.01,2.0)
toolbox.register("attr_zmax", random.uniform,0.01,2.0)
toolbox.register("attr_vmax", random.uniform,0.01,2.0)


# Structure initializers
#                         define 'individual' to be an individual
#                         consisting of 100 'attr_bool' elements ('genes')
toolbox.register("individual", tools.initCycle, creator.Individual, 
    (toolbox.attr_points,toolbox.attr_xmin,toolbox.attr_xmax,toolbox.attr_ymin,toolbox.attr_ymax,toolbox.attr_zmin,toolbox.attr_zmax,toolbox.attr_vmax), n=1)

# define the population to be a list of individuals
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# the goal ('fitness') function to be maximized
#THIS IS THE EVALUATION FUNCTION CALLS MADE FROM HERE TO PYTHON MOVEIT INTERFACE, RETURN Euclidean ERROR 
def evalOneMax(individual):
    #print individual
    #print "individuals vals" 
    #Something like [9, 1.6477799299327085, 1.9630836409254298, 1.309408122302932, 0.30338760434706497, 1.9753449959651421, 1.411187743367649, 1.609950026413625]

    return (sum(individual)), # want to return a Eucidean

#----------
# Operator registration
#----------
# register the goal / fitness function
toolbox.register("evaluate", evalOneMax)

# register the crossover operator
toolbox.register("mate", tools.cxTwoPoint)

# register a mutation operator with a probability to
# flip each attribute/gene of 0.05
toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)

# operator for selecting individuals for breeding the next
# generation: each individual of the current generation
# is replaced by the 'fittest' (best) of three individuals
# drawn randomly from the current generation.
toolbox.register("select", tools.selTournament, tournsize=10)

#----------

def main():
    random.seed(64)

    # create an initial population of 16 individuals (where
    # each individual is a list of integers)
    pop = toolbox.population(n=16)

    # CXPB  is the probability with which two individuals
    #       are crossed
    #
    # MUTPB is the probability for mutating an individual
    CXPB, MUTPB = 0.5, 0.2
    
    print("Start of evolution")
    
    # Evaluate the entire population
    #print pop # long list of tuples
    #THE LINE BELOW CALLS THE EVAUALTE FUNCTION 
    fitnesses = list(map(toolbox.evaluate, pop)) 
    #print fitnesses
    #print "FITNESS ABOVE"
    #self.wvalues = tuple(map(mul, values, self.weights))

    for ind, fit in zip(pop, fitnesses): # Only Called Once 
        ind.fitness.values = fit
    
    print("  Evaluated %i individuals" % len(pop))

    # Extracting all the fitnesses of 
    fits = [ind.fitness.values[0] for ind in pop]

    # Variable keeping track of the number of generations
    g = 0
    
    # Begin the evolution
    while max(fits) < 100 and g < 1000: # Do this while the largest fitness of largest individual in pop is less than 100 and tries is less than 1000
        # A new generation
        g = g + 1
        print("-- Generation %i --" % g)
        
        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))
    
        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):

            # cross two individuals with probability CXPB
            if random.random() < CXPB:
                toolbox.mate(child1, child2)

                # fitness values of the children
                # must be recalculated later
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:

            # mutate an individual with probability MUTPB
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values
    
        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        
        print("  Evaluated %i individuals" % len(invalid_ind))
        
        # The population is entirely replaced by the offspring
        pop[:] = offspring
        
        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]
        
        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x*x for x in fits)
        std = abs(sum2 / length - mean**2)**0.5
        
        print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        print("  Avg %s" % mean)
        print("  Std %s" % std)
    
    print("-- End of (successful) evolution --")
    
    best_ind = tools.selBest(pop, 1)[0]
    print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))

    #https://deap.readthedocs.io/en/master/tutorials/basic/part3.html

if __name__ == "__main__":
    main()

