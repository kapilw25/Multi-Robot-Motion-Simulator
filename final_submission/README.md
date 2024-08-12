# Project Name
```bash
CMPE 297: Multi-Robot Motion Simulator
by Kapil Wanaskar
016649880
```

## Setup Instructions
Follow these steps to get the project up and running on your local machine.

### Prerequisites
- An Anaconda installation. If you do not have Anaconda installed, download it from [Anaconda's site](https://www.anaconda.com/products/distribution).

### Setting Up Your Environment

#### Create and activate a new Conda environment
```bash
# Create a new Conda environment
conda create --name simulator python=3.8
# Activate the environment
conda activate simulator
```

#### Install required packages
```bash
# Install packages from requirements.txt
pip install -r requirements.txt
```

### Running the Application
```bash
# Run the application
streamlit run app.py
```

#### Interacting with the Application
- The application includes several sliders in the sidebar for customizing the simulation settings:
    - **Number of Nodes**: Adjust the number of nodes in the simulation.
    - **X-axis Limits**: Set the minimum and maximum values for the X-axis.
    - **Y-axis Limits**: Set the minimum and maximum values for the Y-axis.
    - **Z-axis Limits**: Set the minimum and maximum values for the Z-axis.
    - **Radius**: Define the radius for each node.
- Utilize these sliders to configure the simulation environment before running the 2D or 3D simulations.
- After adjusting the sliders, click on `Run 2D Simulation` or `Run 3D Simulation` buttons to view the respective animations.


### Project Structure
```
final_submission/
├── README.md               # This file.
├── app.py                  # Main application script.
├── requirements.txt        # Python dependencies.
└── videos                  # Directory of various versions of videos generated while devloping this app's final / 12th version
```
