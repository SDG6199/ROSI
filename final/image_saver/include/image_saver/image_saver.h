#ifndef Image_saver_h
#define Image_saver_h

const std::string OPENCV_WINDOW = "detect line window";
extern const int FIX_WIDTH;
extern const int FIX_HEIGHT;
extern int ORG_X;
extern int ORG_Y;

struct Data_position
{
  int x=0;
  int y=0;
};
extern std::vector<std::vector<Data_position>> no_cross_mat;

class ImageSaver
{
  ros::NodeHandle n_;
  ros::NodeHandlePtr nh_ = boost::make_shared<ros::NodeHandle>();
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_;
  ros::Subscriber save_sub;
  ros::Subscriber box_sub_;
  ros::Subscriber buzzer_stop_sub_;
    
  public:
    ImageSaver(): it_(n_)      //it_에 nh_을 대입한다. 
    {
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageSaver::imageCb, this, image_transport::TransportHints("compressed"));  //멤버함수를 가리키는 함수 포인터는 &ImageSaver::imageCb로 쓴다. 
      save_sub = nh_->subscribe("/image_save_tag",1, &ImageSaver::savetag,this);
      box_sub_ = nh_->subscribe("/darknet_ros/bounding_boxes", 1, &ImageSaver::detect_box, this); //이 template는 호출한 객체의 주소도 전달해야한다. 생성자 내에 있어서 그런거같다.
      buzzer_stop_sub_ = nh_->subscribe("/buzzer_stop", 1, &ImageSaver::buzzer_stop_cb, this); //이 template는 호출한 객체의 주소도 전달해야한다. 생성자 내에 있어서 그런거같다.
      
      //cv::namedWindow(OPENCV_WINDOW);
      //cv::resizeWindow(OPENCV_WINDOW, 400,450);
      //cv::moveWindow(OPENCV_WINDOW, 1920, 0);   //내 모니터 해상도: 1920x1080
    }
    ~ImageSaver()
    {
      //cv::destroyWindow(OPENCV_WINDOW);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void savetag(const std_msgs::Byte::ConstPtr& msg);
    void detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void buzzer_stop_cb(const std_msgs::Byte::ConstPtr& msg);
};

class DetectLine
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber box_sub_;
  ros::Subscriber direction_roi_sub_;

  public:
    DetectLine(): it_(nh_)
    {
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &DetectLine::imageCb, this, image_transport::TransportHints("compressed"));
      box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &DetectLine::draw_box, this);
      direction_roi_sub_ = nh_.subscribe("/direction_roi", 1000, &DetectLine::direction_roi_cb, this);
      
      cv::namedWindow(OPENCV_WINDOW);
      cv::resizeWindow(OPENCV_WINDOW, FIX_WIDTH,FIX_HEIGHT);
    }
    ~DetectLine()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
    void imageCb(const sensor_msgs::ImageConstPtr&);
    void image_process(cv::Mat);
    cv::Mat drawHoughLines(cv::Mat&,std::vector<cv::Vec4i>);
    void draw_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr&);
    void direction_roi_cb(const std_msgs::Byte::ConstPtr&);
};
void split_left_right(std::vector<cv::Vec4i>, std::vector<cv::Vec4i>&,std::vector<cv::Vec4i>&);
void find_line(cv::Mat&,cv::Mat&,std::vector<cv::Vec4i>&,std::vector<cv::Vec4i> &);
bool find_line_params(std::vector<cv::Vec4i>&, float* , float*);

void check_man_illegal();
float calc_distance(Data_position*, darknet_ros_msgs::BoundingBox );

#endif
