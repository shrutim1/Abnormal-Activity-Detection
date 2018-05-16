# Abnormal-Activity-Detection
This project deals with detection of abnormal activity using Intel RealSense R200 camera. 
1.The motiontemplate.cpp which is used for making the MHI templates.

2.PersonTrackingSample.cpp contains code for skeleton points detection and code for 3 cases
1. Mask 
2. Velocity above a certain threahold if this case is detected it writes abnormal to a text file.
3. Two persons detected
4.sound_play package has been used to play audio files in each of the three cases

3.vgg_predict.py and vgg_train.py have code for prediction and training the VGG-16 respectively.
  After the prediction the classification is written to a text file.

4. The bag_to_image.py contains code for getting images from the bag file created.

5. RunBagfileInBackground.sh contains bash script for continuously recording the bag file in the background and creating the MHI templates. After creation of template the script gives input to vgg_predict.py. If result from both skeleton and MHI gives abnormal, the upload python file is run to upload the image to S3 bucket.

6. The MainIntegration.sh bash calls RunBagfileInBackground.sh and launches the skeleton tracking code. 
