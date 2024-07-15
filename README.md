# E-POSE: A Large Scale Event Camera Dataset for Object Pose Estimation
This repository is for the E-POSE dataset for object pose estimation using the event-based cameras. The repository contains the automatic annotation scripts used "CreateSaveDataset", "CreateSaveDatasetMulti" and "CreateSaveDatasetfive" for the dataset to associate the events with the poses of the object.

The files "T_obj_Single objects", "T_obj_five objects" and "T_obj_three objects"   contain the 4x4 transformation matrix between the object and the base frame, used for generating the groundtruth. 

"getMask" and "getMaskMultiple" are the files that generate the mask of the object based on the pose reference to the camera frame.

"RGBTransformation" generates the groundtruth for the RGB images in the bagfile.

All the scripts are currently formatted to extract the data from the .bag files and generate the spike images, masks, poses, 3D bouding boxes and 3-D event images. The scripts will be adjusted to work directly with h5 files provided which contain the raw event and pose streams obtained during the data collection. 
