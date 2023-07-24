"""

Overview
---------

This modules implements some classes helping to test the pycapacity library.

For the moment, it contains the following classes:
    - FourLinkRobot: a class implementing the forward kinematics and jacobian matrix of a four link planar robot

"""
import numpy as np

"""
Four link planar robot example
Direct kinematics and jacobian matrix calculation for the simple 4 dof planar robot
n=4 and m=2
"""

class FourLinkRobot:
    # jacobian function for four link planar robot
    def jacobian(self,joints):
        """
        Function calculating the jacobian matrix of the four link planar robot

        Parameters
        ----------
        joints : numpy.array - joint angles

        Returns
        -------
        J : numpy.array - jacobian matrix
        """

        sq1 = np.sin(joints[0])
        sq12 = np.sin(joints[0] + joints[1])
        sq123 = np.sin(joints[0] + joints[1] + joints[2])
        sq1234 = np.sin(joints[0] + joints[1] + joints[2] + joints[3])
        cq1 = np.cos(joints[0])
        cq12 = np.cos(joints[0] + joints[1])
        cq123 = np.cos(joints[0] + joints[1] + joints[2])
        cq1234 = np.cos(joints[0] + joints[1] + joints[2] + joints[3])
        return np.array([[0.5*cq1+0.5*cq12+0.5*cq123+0.3*cq1234, 0.5*cq12+0.5*cq123+0.3*cq1234, 0.5*cq123+0.7*cq1234, +0.3*cq1234], [-0.5*sq1-0.5*sq12-0.5*sq123-0.3*sq1234, -0.5*sq12-0.5*sq123-0.3*sq1234, -0.5*sq123-0.3*sq1234, -0.3*sq1234]])
    # inertia matrix of a four link planar robot
    def inertia(self,joints):

        """
        Function calculating the inertia matrix of the four link planar robot

        Parameters
        ----------
        joints : numpy.array - joint angles

        Returns
        -------
        M : numpy.array - inertia matrix
        """

        sq1 = np.sin(joints[1])
        cq1 = np.cos(joints[1])
        sq2 = np.sin(joints[2])
        cq2 = np.cos(joints[2])
        sq3 = np.sin(joints[3])
        cq3 = np.cos(joints[3])
        return np.reshape([cq1*(5.0/8.0)+cq2*(3.0/8.0)+cq3/8.0+cq1*cq2*(3.0/8.0)+(cq2*cq3)/8.0-sq1*sq2*(3.0/8.0)-(sq2*sq3)/8.0+(cq1*cq2*cq3)/8.0-(cq1*sq2*sq3)/8.0-(cq2*sq1*sq3)/8.0-(cq3*sq1*sq2)/8.0+7.0/8.0,cq1*(5.0/1.6e+1)+cq2*(3.0/8.0)+cq3/8.0+cq1*cq2*(3.0/1.6e+1)+(cq2*cq3)/8.0-sq1*sq2*(3.0/1.6e+1)-(sq2*sq3)/8.0+(cq1*cq2*cq3)/1.6e+1-(cq1*sq2*sq3)/1.6e+1-(cq2*sq1*sq3)/1.6e+1-(cq3*sq1*sq2)/1.6e+1+1.5e+1/3.2e+1,cq2*(3.0/1.6e+1)+cq3/8.0+cq1*cq2*(3.0/1.6e+1)+(cq2*cq3)/1.6e+1-sq1*sq2*(3.0/1.6e+1)-(sq2*sq3)/1.6e+1+(cq1*cq2*cq3)/1.6e+1-(cq1*sq2*sq3)/1.6e+1-(cq2*sq1*sq3)/1.6e+1-(cq3*sq1*sq2)/1.6e+1+3.0/1.6e+1,cq3/1.6e+1+(cq2*cq3)/1.6e+1-(sq2*sq3)/1.6e+1+(cq1*cq2*cq3)/1.6e+1-(cq1*sq2*sq3)/1.6e+1-(cq2*sq1*sq3)/1.6e+1-(cq3*sq1*sq2)/1.6e+1+1.0/3.2e+1,cq1*(5.0/1.6e+1)+cq2*(3.0/8.0)+cq3/8.0+cq1*cq2*(3.0/1.6e+1)+(cq2*cq3)/8.0-sq1*sq2*(3.0/1.6e+1)-(sq2*sq3)/8.0+(cq1*cq2*cq3)/1.6e+1-(cq1*sq2*sq3)/1.6e+1-(cq2*sq1*sq3)/1.6e+1-(cq3*sq1*sq2)/1.6e+1+1.5e+1/3.2e+1,cq2*(3.0/8.0)+cq3/8.0+(cq2*cq3)/8.0-(sq2*sq3)/8.0+1.5e+1/3.2e+1,cq2*(3.0/1.6e+1)+cq3/8.0+(cq2*cq3)/1.6e+1-(sq2*sq3)/1.6e+1+3.0/1.6e+1,cq3/1.6e+1+(cq2*cq3)/1.6e+1-(sq2*sq3)/1.6e+1+1.0/3.2e+1,cq2*(3.0/1.6e+1)+cq3/8.0+cq1*cq2*(3.0/1.6e+1)+(cq2*cq3)/1.6e+1-sq1*sq2*(3.0/1.6e+1)-(sq2*sq3)/1.6e+1+(cq1*cq2*cq3)/1.6e+1-(cq1*sq2*sq3)/1.6e+1-(cq2*sq1*sq3)/1.6e+1-(cq3*sq1*sq2)/1.6e+1+3.0/1.6e+1,cq2*(3.0/1.6e+1)+cq3/8.0+(cq2*cq3)/1.6e+1-(sq2*sq3)/1.6e+1+3.0/1.6e+1,cq3/8.0+3.0/1.6e+1,cq3/1.6e+1+1.0/3.2e+1,cq3/1.6e+1+(cq2*cq3)/1.6e+1-(sq2*sq3)/1.6e+1+(cq1*cq2*cq3)/1.6e+1-(cq1*sq2*sq3)/1.6e+1-(cq2*sq1*sq3)/1.6e+1-(cq3*sq1*sq2)/1.6e+1+1.0/3.2e+1,cq3/1.6e+1+(cq2*cq3)/1.6e+1-(sq2*sq3)/1.6e+1+1.0/3.2e+1,cq3/1.6e+1+1.0/3.2e+1,1.0/3.2e+1],[4,4]);

    def forward_kinematics(self,joints):
        """
        Function for calculating the forward kinematics of the four link planar robot

        Parameters
        ----------
        joints : numpy.array - joint angles

        Returns
        -------
        x : numpy.array - an array of positions of the end-effector
        """    
        return self.joints_forward_kinematics(joints)[:,-1]

    def joints_forward_kinematics(self, joints):
        """
        Function for calculating the forward kinematics of the four link planar robot

        Parameters
        ----------
        joints : numpy.array - joint angles

        Returns
        -------
        x : numpy.array - an array of positions of each joint + end effector
        """
        L = [0, 0.5,0.5,0.5,0.3]
        x = np.zeros((2,1))
        for i in range(5):
            sq = np.sum(joints[:i])
            x = np.hstack((x, x[:,-1].reshape(2,1)+ L[i]*np.array([[np.sin(sq)], [np.cos(sq)]])));
        return x


    def plot(self,plt, q):
        """
        Plot the four link robot in the current figure

        Parameters
        ----------
        plt : matplotlib.pyplot - current figure
        q : numpy.array - joint angles
        """        
        robot_position = self.joints_forward_kinematics(q)
        plt.plot(robot_position[0,:],robot_position[1,:],linewidth=5, label="robot", marker='o', markerfacecolor='k', markersize=10)
        plt.plot(robot_position[0,0],robot_position[1,0]-0.08,'ks',markersize=20)