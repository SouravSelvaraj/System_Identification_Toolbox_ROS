#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import statsmodels.api as sm
import csv
import math
def generate_sample_data(num_samples):
    t = np.linspace(0, 10, num_samples)  # Generate time points
    motor_position = np.sin(3*t) + np.cos(t) + np.exp(t) # Generate motor position data using a sine wave
    return t, motor_position


class SystemIdentificationNode:
    def __init__(self):
        rospy.init_node('system_identification_node', anonymous=True)
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        self.model_params_pub = rospy.Publisher('model_parameters', Float64, queue_size=10)
        self.accuracy_pub = rospy.Publisher('model_accuracy', Float64, queue_size=10)
        self.data_source = rospy.get_param('~data_source', 'ROS')  # Default to ROS data
        self.selected_model = rospy.get_param('~selected_model', 'AR')  # Default to AR model
        self.order = rospy.get_param('~order', 2)  # Default order
        self.y = []  # store received data for system identification
        self.t = []  # store time data for ARX, ARMAX, ARARX, and ARARMAX models

        if self.selected_model not in ['AR', 'ARX', 'ARMAX', 'ARARX', 'ARARMAX']:
            rospy.logerr("Invalid model selected. Defaulting to AR model.")
            self.selected_model = 'AR'

        if self.data_source == 'CSV':
            self.load_data_from_csv()
        elif self.data_source == 'Random':
            self.generate_random_sample_data()

    def load_data_from_csv(self):
        # Load data from CSV file
        # Replace 'data.csv' with your CSV file path
        with open('data.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                self.y.append(float(row[0]))
                self.t.append(float(row[1]))  # Append time data

    def generate_random_sample_data(self):
        # Generate random sample data
        num_samples = 100  # Number of samples
        self.t, self.y = generate_sample_data(num_samples) # Generate time data

    def joint_state_callback(self, msg):
        # Process received joint state data
        # For demonstration purposes, let's assume we're using position data of the first joint
        if msg.position:
            motor_position = msg.position[0]  # Assuming the first element contains motor position
            self.y.append(motor_position)
            self.t.append(rospy.get_time())  # Capture current time

    def fit_model(self):
        if len(self.y) > 0:
            # Convert time data to numpy array
            t = np.array(self.t)
            if self.selected_model == 'AR':
                order_ar = self.order  # order of the AR model
                model_ar = sm.tsa.AR(self.y)
                results_ar = model_ar.fit(maxlag=order_ar)
                model_params = results_ar.params
            elif self.selected_model == 'ARX':
                order_arx = self.order  # order of the ARX model
                exog = np.column_stack([self.y[:-1], self.t[1:]])  # Use previous motor position and current time as exogenous variables
                endog = self.y[1:]  # Shifted by one to align with exog
                model = sm.OLS(endog, exog)  # Fit ARX model using ordinary least squares
                results = model.fit(maxlag=order_arx)
                model_params = results.params
            elif self.selected_model == 'ARMAX':
                order_armax = self.order  # ARMAX(p, d, q) order
                exog = np.column_stack([self.y[:-1], self.t[1:],np.ones_like(self.y[:-1])])  # Use previous motor position and current time as exogenous variables
                endog = self.y[1:]  # Shifted by one to align with exog
                model = sm.OLS(endog, exog)  # Fit ARMAX model using ordinary least squares
                results = model.fit(maxlag=order_armax)
                model_params = results.params
            elif self.selected_model == 'ARARX':
                order_ararx = self.order  # ARIMAX(p, d, q) order with AR
                exog = np.column_stack([self.y[:-1], self.t[1:], self.y[:-1] * self.t[1:]])  # Use previous motor position, current time, and their interaction as exogenous variables
                endog = self.y[1:]  # Shifted by one to align with exog
                model = sm.OLS(endog, exog)  # Fit ARARX model using ordinary least squares
                results = model.fit(maxlag=order_ararx)
                model_params = results.params
            elif self.selected_model == 'ARARMAX':
                order_ararmax = self.order  # ARIMAX(p, d, q) order with AR
                exog = np.column_stack([self.y[:-1], self.t[1:], self.y[:-1] * self.t[1:],np.ones_like(self.y[:-1])])  # Use previous motor position, current time, and their interaction as exogenous variables
                endog = self.y[1:]  # Shifted by one to align with exog
                model = sm.OLS(endog, exog)  # Fit ARARMAX model using ordinary least squares
                results = model.fit(maxlag=order_ararmax)
                model_params = results.params

            # Calculate accuracy metrics
            predicted_values = results.predict(exog)
            mae = np.mean(np.abs(endog - predicted_values))
            mse = np.mean((endog - predicted_values)**2)
            # Publish model accuracy metrics to ROS topic
            self.accuracy_pub.publish(Float64(mae))
            self.accuracy_pub.publish(Float64(mse))

            # Publish model parameters to ROS topic
            self.model_params_pub.publish(Float64(model_params[0]))
            print(self.order)
            print(model_params)
            print(mse,mae)
            # Plot input data and predicted data
            plt.figure()
            plt.plot(t[1:], endog, label='Input Data')
            plt.plot(t[1:], predicted_values, label='Predicted Data')
            plt.xlabel('Time')
            plt.ylabel('Motor Position')
            plt.title('Input Data vs. Predicted Data')
            plt.legend()
            plt.show()

            

if __name__ == '__main__':
    try:
        system_id_node = SystemIdentificationNode()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            system_id_node.fit_model()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
