# System Identification System

## Overview

The System Identification System is a tool designed to identify and model the behavior of a motor system using various time series analysis techniques, including AR, ARX, ARMAX, ARARX, and ARARMAX models. The system can be integrated with ROS for real-time data acquisition and processing.

## Features

- Supports multiple system identification models: AR, ARX, ARMAX, ARARX, and ARARMAX.
- Integration with ROS for data input from ROS nodes.
- Option to generate sample data for testing and demonstration purposes.
- Model fitting and evaluation functionalities, including accuracy calculation and visualization.

## Dependencies

- Python 3
- NumPy
- SciPy
- Statsmodels
- Matplotlib
- ROS Noetic

## Installation

1. Create and Configure ROS workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Clone the repository:

```bash
git clone https://github.com/SouravSelvaraj/System_Identification_Toolbox_ROS.git
```

3. Caktin Build

```bash
cd ..
catkin build
Source devel/setup.bash
```

## Usage

1. Run the system identification node:

```bash
roslaunch system_iden System_identification.launch data_source:=Random selected_model:=ARX order:=3
```

2. Follow the prompts to select the order, model type (AR, ARX, ARMAX, ARARX, ARARMAX )and data source (CSV file, ROS node, or random sample data).
3. View the model accuracy and predicted data visualization.
4. Optionally, integrate with ROS for real-time data acquisition and processing.

## Configuration

- Adjust model parameters (e.g., number of samples) in the `sysid.py` script as needed.
- Change generate_sample_data function in the `[sysid.py](http://sysid.py)` to change the randomly generated data
- Modify ROS topics and message types for data input and output according to your ROS configuration.

## Acknowledgements

- This project utilizes the [Statsmodels](https://www.statsmodels.org/stable/index.html) library for time series analysis.

## Authors

- [Sourav Selvaraj](https://www.linkedin.com/in/souravselvaraj/)
