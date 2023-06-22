## Traffic sign detection with Duckiebot
This project is a fork of the duckietown/template-ros repository that is a boilerplate repository for developing ROS-based software in Duckietown.

## How to use it
1. Clone this repository
2. Make sure you can connect to the duckiebot. Check with
   `ping [your_duckiebot_name].local`
3. Build the project using the following command in the top directory of the repository:
   `dts devel build -f -H [your_duckiebot_name].local`
4. Run the project on the Duckiebot using:
   `dts devel run -H [your_duckiebot_name].local`
## Additions over the template
The dataset that we trained the YOLOv5 model on is larger than 100 mb, hence we were not able to push it to Github. However, our images are available as described in the next steps. We trained a nano YOLOv5 model using Google Colab for 100 epochs, the notebook used for training can be found in the directory "/ros-six/assets/training-notebook.ipynb". The dataset contains 4 traffic signs and duckies. The 4 traffic signs are: turn right ahead, turn left ahead, 20km speed limit and stop. The traffic signs are random images from the internet. 

We obtained a dataset of traffic signs from Roboflow that can be accessed [https://universe.roboflow.com/usmanchaudhry622-gmail-com/traffic-and-road-signs].

We extended it with images of the printed traffic signs taken by our phones and by the Duckiebot camera itself. We annotated our images with Roboflow. Our annotated images are available here [https://universe.roboflow.com/whatever-7klrp/traffic-sign-hmdah/dataset/1].

The dataset was further extended with images of duckies from the Duckietown project.

## Packages
The object detection packages contains the source code for the object detection node, which is where the main processing occurs.

The nn_model package contains the code and weights needed for the model.
