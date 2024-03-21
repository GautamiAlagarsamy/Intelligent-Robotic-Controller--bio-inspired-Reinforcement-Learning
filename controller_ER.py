from controller import Robot, Receiver, Emitter
import sys,struct,math
import numpy as np
import mlp as ntw

class Controller:
    def __init__(self, robot):
        # Robot Parameters
        # Please, do not change these parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 5  # m/s

        # MLP Parameters and Variables

        ###########
        ### DEFINE below the architecture of your MLP network:

        ### Add the number of neurons for input layer, hidden layer and output layer.
        ### The number of neurons should be in between of 1 to 20.
        ### Number of hidden layers should be one or two.

        self.number_input_layer = 8
        # Example with one hidden layers: self.number_hid_layer = [5]
        # Example with two hidden layers: self.number_hid_layer = [7,5]
        self.number_hid_layer = [12,10,8]
        self.number_op_layer = 2

        # Create a list with the number of neurons per layer
        self.number_neuros_per_layer = []
        self.number_neuros_per_layer.append(self.number_input_layer)
        self.number_neuros_per_layer.extend(self.number_hid_layer)
        self.number_neuros_per_layer.append(self.number_op_layer)

        # Initialize the network
        self.network = ntw.MLP(self.number_neuros_per_layer)
        self.inputs = []

        # Calculate the number of weights of your MLP
        self.num_weight = 0
        for n in range(1,len(self.number_neuros_per_layer)):
            if(n == 1):
                # Input + bias
                self.num_weight += (self.number_neuros_per_layer[n-1]+1)*self.number_neuros_per_layer[n]
            else:
                self.num_weight += self.number_neuros_per_layer[n-1]*self.number_neuros_per_layer[n]

        # Enable Motors
        self.left_motor = self.robot.getDev('left wheel motor')
        self.right_motor = self.robot.getDev('right wheel motor')
        self.left_motor.setPos(float('inf'))
        self.right_motor.setPos(float('inf'))
        self.left_motor.setVel(0.0)
        self.right_motor.setVel(0.0)
        self.vel_left = 0
        self.vel_right = 0

        # Enable Proximity Sensors
        self.prox_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.prox_sensors.append(self.robot.getDev(sensor_name))
            self.prox_sensors[i].enable(self.time_step)

        # Enable Ground Sensors
        self.left_ir = self.robot.getDev('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDev('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDev('gs2')
        self.right_ir.enable(self.time_step)

        # Enable Emitter and Receiver (to communicate with the Supervisor)
        self.emitter = self.robot.getDev("emitter")
        self.receiver = self.robot.getDev("receiver")
        self.receiver.enable(self.time_step)
        self.rec_Data = ""
        self.rec_Data_Previous = ""
        self.flag_Message = False
        
        # Enable Light Sensors
        self.l_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            self.l_sensors.append(self.robot.getDev(sensor_name))
            self.l_sensors[i].enable(self.time_step)

        # Fitness value (initialization fitness parameters once)
        self.fitness_val = []
        self.fitness = 0
        
        #Variable to determine if the beacon is activated
        self.tmpLight = 0 

    def check_for_new_genes(self):
        if(self.flag_Message == True):
                self.tmpLight = 0 #Value reset to 0 at the beginning of each simulation the beacon is off
                
                # Split the list based on the number of layers of your network
                part = []
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        part.append((self.number_neuros_per_layer[n-1]+1)*(self.number_neuros_per_layer[n]))
                    else:
                        part.append(self.number_neuros_per_layer[n-1]*self.number_neuros_per_layer[n])

                # Set the weights of the network
                data = []
                weightsPart = []
                sum = 0
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        weightsPart.append(self.rec_Data[n-1:part[n-1]])
                    elif(n == (len(self.number_neuros_per_layer)-1)):
                        weightsPart.append(self.rec_Data[sum:])
                    else:
                        weightsPart.append(self.rec_Data[sum:sum+part[n-1]])
                    sum += part[n-1]
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        weightsPart[n-1] = weightsPart[n-1].reshape([self.number_neuros_per_layer[n-1]+1,self.number_neuros_per_layer[n]])
                    else:
                        weightsPart[n-1] = weightsPart[n-1].reshape([self.number_neuros_per_layer[n-1],self.number_neuros_per_layer[n]])
                    data.append(weightsPart[n-1])
                self.network.weights = data

                #Reset fitness list
                self.fitness_val = []
                
                

    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self):
        # MLP:
        #   Input == sensory data
        #   Output == motors commands
        output = self.network.propagate_forward(self.inputs)
        self.vel_left = output[0]
        self.vel_right = output[1]

        # Multiply the motor values by 3 to increase the velocities
        self.left_motor.setVel(self.vel_left*5)
        self.right_motor.setVel(self.vel_right*5)

    def calculate_fitness(self):

        forwardFitness = (self.vel_left+self.vel_right)/2
        #print("forwardFitness : {}" .format(forwardFitness))
        
        ### Define the fitness function to encourage the robot to follow the line
        followLineFitness = 1 - np.sum(self.inputs[0:3])/3
        #print("followLineFitness : {}".format(followLineFitness))
        
        ### Define the fitness function to avoid collision
        avoidCollisionFitness = 1 - max(self.inputs[3:7])
        #print("avoidCollisionFitness : {}".format(avoidCollisionFitness))
        
        ### Define the fitness function to avoid spining behaviour
        spinningFitness = 1-math.sqrt(math.pow((self.vel_right - self.vel_left),2))
        #print("spinningFitness : {}".format(spinningFitness))
        
        ### Define the fitness function of this iteration which should be a combination of the previous functions         
        combinedFitness = forwardFitness * spinningFitness * avoidCollisionFitness * followLineFitness 
        #print("combinedFitness : {}".format(combinedFitness))
        self.fitness_val.append(combinedFitness)
        self.fitness = np.mean(self.fitness_val)

    def handle_emitter(self):
        # Send the self.fitness value to the supervisor
        data = str(self.num_weight)
        data = "weights: " + data
        string_msg = str(data)
        string_msg = string_msg.encode("utf-8")
        #print("Robot send:", string_msg)
        self.emitter.send(string_msg)

        # Send the self.fitness value to the supervisor
        data = str(self.fitness)
        data = "fitness: " + data
        string_msg = str(data)
        string_msg = string_msg.encode("utf-8")
        #print("Robot send fitness:", string_msg)
        self.emitter.send(string_msg)

    def handle_receiver(self):
        if self.receiver.getQueueLength() > 0:
            while(self.receiver.getQueueLength() > 0):
                # Adjust the Data to our model
                #Webots 2022:
                #self.rec_Data = self.receiver.getData().decode("utf-8")
                #Webots 2023:
                self.rec_Data = self.receiver.getString()
                self.rec_Data = self.rec_Data[1:-1]
                self.rec_Data = self.rec_Data.split()
                x = np.array(self.rec_Data)
                self.rec_Data = x.astype(float)
                #print("Controller handle receiver data:", self.rec_Data)
                self.receiver.nextPacket()

            # Is it a new Genotype?
            if(np.array_equal(self.rec_Data_Previous,self.rec_Data) == False):
                self.flag_Message = True

            else:
                self.flag_Message = False

            self.rec_Data_Previous = self.rec_Data
        else:
            #print("Controller receiver q is empty")
            self.flag_Message = False

    def run_robot(self):
        # Main Loop
        while self.robot.step(self.time_step) != -1:
            # This is used to store the current input data from the sensors
            self.inputs = []

            # Emitter and Receiver
            # Check if there are messages to be sent or read to/from our Supervisor
            self.handle_emitter()
            self.handle_receiver()

            # Read Ground Sensors
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()
            #print("Ground Sensors \n    left {} center {} right {}".format(left,center,right))

            ### Please adjust the ground sensors values to facilitate learning : 0 - 1023
            min_gs = 10
            max_gs = 1022

            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs

            # Normalize the values between 0 and 1 and save data
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))
            #print("Ground Sensors \n    left {} center {} right {}".format(self.inputs[0],self.inputs[1],self.inputs[2]))

            # Read Distance Sensors
            for i in range(8):
                ### Select the distance sensors that you will use
                if(i==0 or i==1 or i==6 or i==7):
                    temp = self.prox_sensors[i].getValue()

                    ### Please adjust the distance sensors values to facilitate learning : 0 - 4095
                    min_ds = 100
                    max_ds = 1000

                    if(temp > max_ds): temp = max_ds
                    if(temp < min_ds): temp = min_ds

                    # Normalize the values between 0 and 1 and save data
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
                    #print("Distance Sensors - Index: {}  Value: {}".format(i,self.prox_sensors[i].getValue()))
                    
            # Read Light Sensors
            for i in range(8):       
                temp = self.l_sensors[i].getValue()
                # Adjust Values : 0-4095
                min_ls = 1000
                max_ls = 3000
                if(temp > max_ls): temp = max_ls
                if(temp < min_ls): temp = min_ls
                # Save Data
                value = (temp-min_ls)/(max_ls-min_ls)
                if(value<0.5):
                    self.tmpLight = 1
            #Append only a boolean value depeding if the beacon is ON or OFF
            self.inputs.append(self.tmpLight)
                #print("Light Sensors - Index: {}  Value: {}".format(i,self.l_sensors[i].getValue()))
            
            # GA Iteration
            # Verify if there is a new genotype to be used that was sent from Supervisor
            self.check_for_new_genes()
            # The robot's actuation (motor values) based on the output of the MLP
            self.sense_compute_and_actuate()
            # Calculate the fitnes value of the current iteration
            self.calculate_fitness()
            
            # End of the iteration

if __name__ == "__main__":
    # Call Robot function to initialize the robot
    my_robot = Robot()
    # Initialize the parameters of the controller by sending my_robot
    controller = Controller(my_robot)
    # Run the controller
    controller.run_robot()

