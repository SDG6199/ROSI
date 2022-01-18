#include <ros/ros.h>
#include <experimental/filesystem> //std::filesystem::path, exists사용. targetlinlibrary에 stdc++fs추가.
#include <iostream>
#include <fstream>   //iostream으로 부터 파생.
#include <stdlib.h>  //system()
//#include <opencv2/imgproc/imgproc.hpp>       //OpenCV's image processing를 사용하기위해 필요하다.
#include <opencv2/highgui/highgui.hpp>       //OpenCV's GUI modules를 사용하기위해 필요하다. (cv::namedWindow(OPENCV_WINDOW);)

using namespace std;
namespace fs = std::experimental::filesystem;

extern int captured_car_count;

int cut_plate()
{
    //ros::Duration(0.1).sleep();
    string pkg_path="/home/sdg/catkin_ws/src/image_saver";
    string buffer;
    ifstream ifs;
    ofstream ofs;
    int left_x,top_y,width,height;

    buffer=pkg_path+"/data/train.txt";
    ofs.open(buffer.c_str());
    if(ofs.is_open())
    {
        buffer=pkg_path+"/image/car_image/captured car"+to_string(captured_car_count)+".jpg";
        ofs.write(buffer.c_str(),buffer.size());
        ofs.close();
    }
    else return 0;
    
    fs::path p(buffer.c_str());
    
    if(fs::exists(p))
    {
        ROS_INFO("captured_car_count: %d \n", captured_car_count);
        //system("cd /home/sdg/catkin_ws/src/image_saver/image/plate_detect_image");  system함수로 cd는 안된다.
        //darknet/src/detector.c 수정 후 make clean & make. 
        
        buffer="darknet detector test"+(" "+pkg_path)+"/data/coco.data"+" "+pkg_path+"/yolo_network_config/cfg/yolov3.cfg"+" "+pkg_path+"/yolo_network_config/weight/yolov3.weights"+
        " "+"-ext_output -dont_show <"+pkg_path+"/data/train.txt"+"> "+pkg_path+"/data/bboxes_detail.txt";
        //darknet detector test /home/sdg/catkin_ws/src/image_saver/data/coco.data /home/sdg/catkin_ws/src/image_saver/yolo_network_config/cfg/yolov3.cfg /home/sdg/catkin_ws/src/image_saver/yolo_network_config/weight/yolov3.weights -ext_output -dont_show </home/sdg/catkin_ws/src/image_saver/data/train.txt> /home/sdg/catkin_ws/src/image_saver/data/bboxes_detail.txt
        system(buffer.c_str()); 
        cv::waitKey(1);

        buffer=pkg_path+"/data/bboxes_detail.txt";
        ifs.open(buffer.c_str());
        if(ifs.is_open())
        {
            string line;
            int blank_index;
            while(!ifs.eof())
            {
                getline(ifs,line);
                if(line.find("car:")!=string::npos)
                {
                    blank_index=line.find(" ",line.find("left_x")+10);  
                    buffer = line.substr(line.find("left_x")+10,blank_index-(line.find("left_x")+10));
                    left_x=stoi(buffer);
                    ROS_INFO("left_x: %d \n", left_x);

                    blank_index=line.find(" ",line.find("top_y")+9);
                    buffer = line.substr(line.find("top_y")+9,blank_index-line.find("top_y")+9);
                    top_y=stoi(buffer);
                    ROS_INFO("top_y: %d \n", top_y);

                    blank_index=line.find(" ",line.find("width")+8);
                    buffer = line.substr(line.find("width")+8,blank_index-line.find("width")+8);
                    width=stoi(buffer);
                    ROS_INFO("width: %d \n", width);

                    blank_index=line.find(" ",line.find("height")+9);
                    buffer = line.substr(line.find("height")+9,blank_index-line.find("height")+9);
                    height=stoi(buffer);
                    ROS_INFO("height: %d \n", height);
                }
            }
            ifs.close();  
        }
        else return 0;
    }
    else return 0;

    buffer=pkg_path+"/image/car_image/captured car"+to_string(captured_car_count)+".jpg";
    cv::Mat image = cv::imread(buffer, cv::IMREAD_UNCHANGED);
    image=image(cv::Rect(left_x,top_y,width,height));
   
    buffer=pkg_path+"/image/plate_cut_image"+"/cut plate"+to_string(captured_car_count)+".jpg";
    cv::imwrite(buffer,image);

    return 1;
}