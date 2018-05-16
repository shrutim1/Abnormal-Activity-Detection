while true
do
	
        dirc="/home/shruti/record"
	if [ -d "$dirc" ]
	then
	rm -r /home/shruti/record
	fi
	mkdir record
	cd record
	rosbag record -a --duration=8
	#kill $pid_launch
	bagfile_name=`ls /home/shruti/record/`
	echo "Bagfile name is : " $bagfile_name
	cp /home/shruti/record/$bagfile_name /home/shruti/catkin_ws/src/my_package/src/
	cd /home/shruti/catkin_ws/src/my_package/src/
	file="./savedImages1"
	if [ -d "$file" ]
	then
		rm -r savedImages1
	fi
	mkdir savedImages1
	cd /home/shruti/catkin_ws/
	. ~/catkin_ws/devel/setup.bash
	rosrun my_package bag_to_image.py savedImages1 $bagfile_name
	cd /home/shruti/opencv-3.1.0/samples/cpp
	g++ -o motiontemp motiontemp.cpp `pkg-config opencv --cflags --libs`
	./motiontemp
	cd /home/shruti/actions/
	convert action.jpg -threshold 1% action_gray.jpg
	template="action_gray.jpg"
	cd /home/shruti/VGGFinal/
	if [ -f "$template" ]
	then
		rm action_gray.jpg
	fi
	mv /home/shruti/actions/action_gray.jpg /home/shruti/VGGFinal/
	python vgg_predict.py action_gray.jpg >foo.txt
	var=$(cat foo.txt)
	b="abnormal"
        cd /home/shruti/catkin_ws
        var1=$(cat skeleton.txt)
        b1="abnormal"
        cd /home/shruti
        if [ "$var" == "$b" -a "$var1" == "$b1" ]
        then
             echo "ABNORMAL ACTIVITY DETECTED! SENDING DATA TO CLOUD...."
	     cp -r /home/shruti/catkin_ws/src/my_package/src/savedImages1 /home/shruti/abormal_actions
	     python 44.py
	     python upload
            # rosrun sound_play say.py "uploading to cloud"
        fi
done
