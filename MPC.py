

import numpy as np

class ModelPredictiveControl(object):
    
    # M_bar, F_bar, tau_d_bar- system matrices
    # f -  prediction horizon
    # v  - control horizon
    # W3 - input weight matrix
    # W4 - prediction weight matrix
    # x0 - initial state of the system
    # desiredControlTrajectoryTotal - total desired control trajectory
    #                               later on, we will take segments of this
    #                               desired state trajectory
    
    def __init__(self,M_bar, F_bar, tau_d_bar,f,v,W3,W4,x0,desiredControlTrajectoryTotal):
        
        # initialize variables
        self.M_bar = M_bar
        self.F_bar = F_bar
        self.tau_d_bar = tau_d_bar
        self.f=f
        self.v=v
        self.W3=W3 
        self.W4=W4
        self.desiredControlTrajectoryTotal=desiredControlTrajectoryTotal

        
        # dimensions of the matrices
        
        self.n=M_bar.shape[0]
        self.r=F_bar.shape[0]
        self.m=tau_d_bar.shape[1]        
                
        # this variable is used to track the current time step k of the controller
        # after every calculation of the control inpu, this variables is incremented for +1 
        self.currentTimeStep=0
        
        # we store the state vectors of the controlled state trajectory
        self.states=[]
        self.states.append(x0)
        
        # we store the computed inputs 
        self.inputs=[]
        
        # we store the output vectors of the controlled state trajectory
        self.outputs=[]
        
        
        # form the lifted system matrices and vectors
        # the gain matrix is used to compute the solution 
        # here we pre-compute it to save computational time
        self.O, self.M, self.gainMatrix = self.formLiftedMatrices()
        
    
    # this function forms the lifted matrices O and M, as well as the 
    # the gain matrix of the control algorithm
    # and returns them 
    def formLiftedMatrices(self):
        f=self.f
        v=self.v
        r=self.r
        n=self.n
        m=self.m
        A=self.M_bar
        B=self.F_bar
        C=self.tau_d_bar
        
        # lifted matrix O
        O=np.zeros(shape=(f*r,n))

        for i in range(f):
            if (i == 0):
                powA=A;
            else:
                powA=np.matmul(powA,A)
            O[i*r:(i+1)*r,:]=np.matmul(C,powA)

        # lifted matrix M        
        M=np.zeros(shape=(f*r,v*m))
    
        for i in range(f):
            # until the control horizon
            if (i<v):
                for j in range(i+1):
                    if (j == 0):
                        powA=np.eye(n,n);
                    else:
                        powA=np.matmul(powA,A)
                    M[i*r:(i+1)*r,(i-j)*m:(i-j+1)*m]=np.matmul(C,np.matmul(powA,B))
            
            # from control horizon until the prediction horizon
            else:
                for j in range(v):
                    # here we form the last entry
                    if j==0:
                        sumLast=np.zeros(shape=(n,n))
                        for s in range(i-v+2):
                            if (s == 0):
                                powA=np.eye(n,n);
                            else:
                                powA=np.matmul(powA,A)
                            sumLast=sumLast+powA
                        M[i*r:(i+1)*r,(v-1)*m:(v)*m]=np.matmul(C,np.matmul(sumLast,B))
                    else:
                        powA=np.matmul(powA,A)
                        M[i*r:(i+1)*r,(v-1-j)*m:(v-j)*m]=np.matmul(C,np.matmul(powA,B))
        
        
        tmp1=np.matmul(M.T,np.matmul(self.W4,M))
        tmp2=np.linalg.inv(tmp1+self.W3)
        gainMatrix=np.matmul(tmp2,np.matmul(M.T,self.W4))
        
        
        return O,M,gainMatrix
    

    def kinematics_model(self, x, u):
        e, phi, alpha = x
        v, omega = u
        
        e_dot = -v * np.cos(alpha)
        phi_dot = v * np.sin(alpha) / e
        alpha_dot = v * np.sin(alpha) / e - omega
        
        return np.array([e_dot, phi_dot, alpha_dot])

    def dynamics_model(self, u, M_bar, F_bar, tau_d_bar):
        tau_1, tau_2 = u
        m, I = M_bar.diagonal()
        
        v_dot = (tau_1 - F_bar[0] - tau_d_bar[0]) / m
        omega_dot = (tau_2 - F_bar[1] - tau_d_bar[1]) / I
        
        return np.array([v_dot, omega_dot])
    
    def propagateDynamics(self,controlInput,state):

        dt = 0.1  # Sampling time, adjust as needed
        e, phi, alpha, v, omega = state
        tau_1, tau_2 = controlInput

        # Kinematics update
        x_dot = self.kinematics_model([e, phi, alpha], [v, omega])
        e += x_dot[0] * dt
        phi += x_dot[1] * dt
        alpha += x_dot[2] * dt

        # Dynamics update
        u_dot = self.dynamics_model([tau_1, tau_2], self.M_bar, self.F_bar, self.tau_d_bar)
        v += u_dot[0] * dt
        omega += u_dot[1] * dt

        new_state = np.array([e, phi, alpha, v, omega])
        output = np.array([e, phi, alpha])  # Assuming these are the observed outputs

        return new_state, output
        
    # this function computes the control inputs, applies them to the system 
    # by calling the propagateDynamics() function and appends the lists
    # that store the inputs, outputs, states
    def computeControlInputs(self):
                
        # extract the segment of the desired control trajectory
        desiredControlTrajectory=self.desiredControlTrajectoryTotal[self.currentTimeStep:self.currentTimeStep+self.f,:]

        # compute the vector s
        vectorS=desiredControlTrajectory-np.matmul(self.O,self.states[self.currentTimeStep])
       
        # compute the control sequence
        inputSequenceComputed=np.matmul(self.gainMatrix,vectorS)
        inputApplied=np.zeros(shape=(1,1))
        inputApplied[0,0]=inputSequenceComputed[0,0]
        
        # compute the next state and output
        state_kp1, output_k = self.propagateDynamics(inputApplied, self.states[self.currentTimeStep])
        
        # append the lists
        self.states.append(state_kp1)
        self.outputs.append(output_k)
        self.inputs.append(inputApplied)
        # increment the time step
        self.currentTimeStep=self.currentTimeStep+1
        


    
    
    
    
    