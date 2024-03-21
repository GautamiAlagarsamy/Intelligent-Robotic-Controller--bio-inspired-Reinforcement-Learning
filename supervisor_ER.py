from controller import Supervisor
from controller import Keyboards
from controller import Display

import numpy as np
import ga,os,sys,struct,math

class SupervisorGA:
    def __init__(self):
        # Simulation Parameters
        # Please, do not change these parameters
        self.time_step = 32 # ms
        self.time_experiment = 60*3 # s
        self.flag_light=0
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Check if the robot node exists in the current world file
        self.robo_node = self.supervisor.getFromDef("Controller")
        if self.robo_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        # Get the robots translation and rotation current parameters    
        self.t_field = self.robo_node.getField("translation")  
        self.r_field = self.robo_node.getField("rotation")
        
        self.light_node = self.supervisor.getFromDef("Light")
        if self.light_node is None:
            sys.stderr.write("No DEF SpotLight node found in the current world file\n")
            sys.exit(1)
        # Get the location and direction fields from light node          
        self.location_field = self.light_node.getField("location")
        self.direction_field = self.light_node.getField("direction")
        
        # Check Receiver and Emitter are enabled
        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(self.time_step)
        
        # Initialize the receiver and emitter data to null
        self.receivedData = "" 
        self.receivedWeights = "" 
        self.receivedFitness = "" 
        self.emitterData = ""
        
        ###########
        ### DEFINE here the 3 GA Parameters:
        self.num_generations = 100f0
        self.num_population = 6
        self.num_elite = 2
        
        # size of the genotype variable
        self.num_weights = 0
        
        # Creating the initial population
        self.population = []
        
        # All Genotypes
        self.genotypes = []
        
        self.path = 0
        self.p1 = 0
        self.p2 = 0 
        self.suc_flag = 0
        self.d1 = []
        self.d2 = []
        self.final_dist = []
        
        # Display: screen to plot the fitness values of the best individual and the average of the entire population
        self.display = self.supervisor.getDevice("display")
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.pre_best_fitness = 0.0;
        self.pre_avg_fitness = 0.0;
        self.display.drawText("Fitness (Best - Red)", 0,0)
        self.display.drawText("Fitness (Average - Green)", 0,10)


    def createRandomPopulation(self):
        # Wait until the supervisor receives the size of the genotypes (number of weights)
        if(self.num_weights > 0):
            #  Size of the population and genotype
            pop_size = (self.num_population,self.num_weights)
            # Create the initial population with random weights
            self.population = np.random.uniform(low=-1.0, high=1.0, size=pop_size)

    def h_receiver(self):
        while(self.receiver.getQueueLength() > 0):
            #Webots 2022: 
            #self.receivedData = self.receiver.getData().decode("utf-8")
            #Webots 2023: 
            self.receivedData = self.receiver.getString()
            typeMessage = self.receivedData[0:7]
            # Check Message 
            if(typeMessage == "weights"):
                self.receivedWeights = self.receivedData[9:len(self.receivedData)] 
                self.num_weights = int(self.receivedWeights)
            elif(typeMessage == "fitness"):  
                self.receivedFitness = float(self.receivedData[9:len(self.receivedData)])
            self.receiver.nextPacket()
    
        #Success function: Determines if the robot managed to accomplish the task no matter the path taken
    def success(self,position):
        x_goal = 0.1
        y_goal = 0.95
        success = 0
        distance = math.sqrt(math.pow((x_goal-position[0]),2)+math.pow((y_goal-position[1]),2))
        self.final_dist.append(distance)
        if distance<0.05:
            self.sucess_flag = 1
            print("Robot achieved goal")
            
    def h_emitter(self):
        if(self.num_weights > 0):
            # Send genotype of an individual
            string_message = str(self.emitterData)
            string_message = string_message.encode("utf-8")
            #print("Supervisor send:", string_message)
            self.emitter.send(string_message)  
    
    #Path detection function : From the light status, and the center coordinates of the obstacles,
    #it computes the distance from robot position and the obstacle.
    #If the robot is within the obstacle's circle; it is considered as validated.
    #In order to succeed in the correct path, the robot must validate the 2 correct obtsacles
    def path_detection(self,position):
        status = self.light_node.getField("on").getSFBool()
        if(status):
            x1_path = -0.23
            y1_path = 0.3
            x2_path = -0.213
            y2_path = 0.692
            
        else:
            x1_path = 0.37
            y1_path = 0.26
            x2_path = 0.38
            y2_path = 0.71
            
        d1 = math.sqrt(math.pow((x1_path-position[0]),2)+math.pow((y1_path-position[1]),2))
        d2 = math.sqrt(math.pow((x2_path-position[0]),2)+math.pow((y2_path-position[1]),2))
        self.d1.append(d1)
        self.d2.append(d2)
        
        if(d1 < 0.13 and not self.p1):
            self.p1 = 1
            print("Good 1st obstacle")
        if(d2 < 0.13 and not self.p2):
            self.p2 = 1
            print("Good 2nd obstacle")
        if(self.p1+self.p2==2):
            self.path=1
            print("Robot took correct path")
    
    #Bonus function: Determines if the correct path was taken and calculate the new fitness from it.
    def bonus(self, fitness, path, success):
        rewards = 0
        penalties = 0
        if(path and success): #Add a rewards of 0.2
           rewards = 0.2
        else: #Compute the avg minimal distance between the 2 obstacles and the final goal
            penalties = (np.min(self.d1)/10+np.min(self.d2)/10+np.min(self.final_dist)/10)/3
           
        fitness += rewards - penalties 
        return fitness
            
        
    def run_seconds(self,seconds):
        #print("Run Simulation")
        stop = int((seconds*1000)/self.time_step)
        iteration = 0
        #Variables to be reset at the beginning of each run
        self.path = 0
        self.p1 = 0
        self.p2 = 0
        self.sucess_flag = 0
        self.d1 = []
        self.d2 = []
        self.final_dist = []
        while self.supervisor.step(self.time_step) != -1:
            self.h_emitter()
            self.h_receiver()
            position=self.t_field.getSFVec3f()
            #print(self.t_field.getSFVec3f())
            if(stop == iteration):
                break
            if(not self.suc_flag):
                self.success(position)
            else:
                break
            if(not self.path):
                self.path_detection(position)
            iteration = iteration + 1
                
    def evaluate_genotype(self,genotype,generation):
        # Here you can choose how many times the current individual will interact with both environments
        # At each interaction loop, one trial on each environment will be performed
        numberofInteractionLoops = 1
        currentInteraction = 0
        fitnessPerTrial = []
        while currentInteraction < numberofInteractionLoops:
            #######################################
            # TRIAL: Beacon activated
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [-0.685987, -0.66, 0]
            self.t_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [0, 0, 1, 1.63]
            self.r_field.setSFRotation(INITIAL_ROT)
            self.robo_node.resetPhysics()
            
            self.light_node.getField("on").setSFBool(True)
        
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness = self.receivedFitness
            
            # Check for Reward and add it to the fitness value here
            # Add your code here
            fitness = self.bonus(fitness, self.path, self.suc_flag)
                       
            print("Fitness: {}".format(fitness))     
                        
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            #######################################
            # TRIAL: Beacon deactivated
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [-0.685987, -0.66, 0]
            self.t_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [0, 0, 1, 1.63]
            self.r_field.setSFRotation(INITIAL_ROT)
            self.robo_node.resetPhysics()
            
            self.light_node.getField("on").setSFBool(False)
        
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness = self.receivedFitness
            
            # Check for Reward and add it to the fitness value here
            # Add your code here
            fitness = self.bonus(fitness, self.path, self.suc_flag)
            
            print("Fitness: {}".format(fitness))
            
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            # End 
            currentInteraction += 1
            
        print(fitnessPerTrial)    
        
        fitness = np.mean(fitnessPerTrial)
        current = (generation,genotype,fitness)
        self.genotypes.append(current)  
        
        return fitness

    def run_demo(self):
        # Read File
        genotype = np.load("Best.npy")
        
        # Turn Left
        
        # Send Genotype to controller
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [-0.685987, -0.66, 0]
        self.t_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [0, 0, 1, 1.63]
        self.r_field.setSFRotation(INITIAL_ROT)
        self.robo_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment) 
        
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness without rewards or penalties: {}".format(fitness))
        
        # Turn Right
        
        # Send Genotype to controller
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [-0.685987, -0.66, 0]
        self.t_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [0, 0, 1, 1.63]
        self.r_field.setSFRotation(INITIAL_ROT)
        self.robo_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment)  
        
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness without rewards or penalties: {}".format(fitness))    
    
    def run_opt(self):
        # Wait until the number of weights is updated
        while(self.num_weights == 0):
            self.h_receiver()
            self.createRandomPopulation()
        
        print(">>>Starting Evolution using GA optimization ...\n")
        
        # For each Generation
        for generation in range(self.num_generations):
            print("Generation: {}".format(generation))
            current_population = []   
            # Select each Genotype or Individual
            for population in range(self.num_population):
                genotype = self.population[population]
                # Evaluate
                fitness = self.evaluate_genotype(genotype,generation)
                #print(fitness)
                # Save its fitness value
                current_population.append((genotype,float(fitness)))
                #print(current_population)
                
            # After checking the fitness value of all indivuals
            # Save genotype of the best individual
            best = ga.getBestGenotype(current_population);
            average = ga.getAverageGenotype(current_population);
            np.save("Best.npy",best[0])
            self.plot_fitness(generation, best[1], average);
            
            # Generate the new population using genetic operators
            if (generation < self.num_generations - 1):
                self.population = ga.population_reproduce(current_population,self.num_elite);
        
        #print("All Genotypes: {}".format(self.genotypes))
        print("GA optimization terminated.\n")   
    
    
    def draw_scaled_line(self, generation, y1, y2): 
        # the scale of the fitness plot
        XSCALE = int(self.width/self.num_generations);
        YSCALE = 100;
        self.display.drawLine((generation-1)*XSCALE, self.height-int(y1*YSCALE), generation*XSCALE, self.height-int(y2*YSCALE));
    
    def plot_fitness(self, generation, best_fitness, average_fitness):
        if (generation > 0):
            self.display.setColor(0xff0000);  # red
            self.draw_scaled_line(generation, self.pre_best_fitness, best_fitness);
    
            self.display.setColor(0x00ff00);  # green
            self.draw_scaled_line(generation, self.pre_avg_fitness, average_fitness);
    
        self.pre_best_fitness = best_fitness;
        self.pre_avg_fitness = average_fitness;
  
    
if __name__ == "__main__":
    # Call Supervisor function to initiate the supervisor module   
    GAModel = SupervisorGA()
    
    # Function used to run the best individual or the GA
    keyboard = Keyboards()
    keyboard.enable(50)
    
    # Interface
    print("***************************************************************************************************")
    print("To start the simulation please click anywhere in the SIMULATION WINDOW(3D Window) and press either:")
    print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
    print("***************************************************************************************************")
    
    while GAModel.supervisor.step(GAModel.time_step) != -1:
        resp = keyboard.getKey()
        if(resp == 83 or resp == 65619):
            GAModel.run_opt()
            print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
            #print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        elif(resp == 82 or resp == 65619):
            GAModel.run_demo()
            print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
            #print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        
