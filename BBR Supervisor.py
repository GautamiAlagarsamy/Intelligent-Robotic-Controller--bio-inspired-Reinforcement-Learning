from controller import Supervisor
import sys
###
### code cite by supervisor_lab2.py https://canvas.hw.ac.uk/courses/25263/files/2862323?module_item_id=1728087
###

class SupervisorLight:
    def __init__(self):
        # Simulation Parameters
        # Simulation Parameters
        self.time_step = 32 # (ms)
        self.time_light = 60 # (s)
        self.flag_light = 1 # You can use the flag to identify the current position of the light node
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Get the robot node from your world environment
        self.robot_node = self.supervisor.getFromDef("Controller")
        # Check if the robot node exists 
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        # Get the rotation and translation fields from your robot node
        self.trans_field = self.robot_node.getField("translation")  
        self.rot_field = self.robot_node.getField("rotation")        
        # Get the light node from your world environment
        self.light_node = self.supervisor.getFromDef("Light")
        if self.light_node is None:
            sys.stderr.write("No DEF SpotLight node found in the current world file\n")
            sys.exit(1)

        self.light_value = True 
        self.light_on_field = self.light_node.getField("on")
        self.light_on_field.setSFBool(self.light_value)
        
    def run_seconds(self,seconds):
        #print("Run Simulation")
        stop = int((seconds*1000)/self.time_step)
        iterations = 0
        while self.supervisor.step(self.time_step) != -1:

            if(stop == iterations):
                # Reset the counter
                iterations = 0
                # Reset physics of the robot (position and rotation)
                # Position
                INITIAL_TRANS = [-0.686,-0.66, -6.3949e-05]
                self.trans_field.setSFVec3f(INITIAL_TRANS)
                # Rotation
                INITIAL_ROT = [8.09129e-05, -7.61133e-05, 1, 1.63194]
                self.rot_field.setSFRotation(INITIAL_ROT)
                self.robot_node.resetPhysics()
                self.light_value = not self.light_value
                self.light_on_field.setSFBool(self.light_value)
                
            iterations += 1          

    def run_demo(self):   
        # Reset robot position and physics
        INITIAL_TRANS = [-0.686, -0.66, -6.3949e-05]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [8.09129e-05, -7.61133e-05, 1, 1.63194]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()  
        # Update the position of the source of light after every 60 s (self.time_light == 60)
        self.run_seconds(self.time_light)
            
if __name__ == "__main__":
    # Create Supervisor Controller
    model = SupervisorLight()
    # Run Supervisor Controller
    model.run_demo()
        
