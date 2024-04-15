#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class NodeLogic{
    public:
        ros::NodeHandle nh;
        ros::Subscriber map_in_sub;
        ros::Publisher map_out_pub;
        double res;
        double occ_thresh = 20;
        NodeLogic(ros::NodeHandle& nh_, double& res_) : nh(nh_), res(res_){
            map_in_sub = ros::Subscriber(nh.subscribe("map", 1, &NodeLogic::map_in_callback, this));
            map_out_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_downsampled", 1, true);
        }
        void map_in_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            // Check incoming map resolution
            if (msg->info.resolution > res){
                ROS_INFO("Incoming map resolution is higher than the target resolution. No downsampling needed.");
                map_out_pub.publish(*msg);
            }
            nav_msgs::OccupancyGrid map_downsampled;
            map_downsampled.header = msg->header;
            map_downsampled.info.resolution = res;
            map_downsampled.info.width = int(msg->info.width * msg->info.resolution / res);
            map_downsampled.info.height = int(msg->info.height * msg->info.resolution / res);
            map_downsampled.info.origin = msg->info.origin;

            int stride = int(res / msg->info.resolution);
            std::cout << "stride: " << stride << std::endl;

            for(int i = 0; i < map_downsampled.info.width; i++){
                for(int j = 0; j < map_downsampled.info.height; j++){
                    // Downsampling logic
                    int free_count = 0; int occ_count = 0; int unk_count = 0;

                    for(int k = 0; k < stride; k++){
                        for(int l = 0; l < stride; l++){
                            int occ_val = msg->data[(i * stride + k) * msg->info.width + j * stride + l];
                            if(occ_val >=0 && occ_val <=occ_thresh){
                                free_count++;
                            }else if(occ_val > occ_thresh){
                                occ_count++;
                            }else{
                                unk_count++;
                            }
                        }
                    }
                    if (occ_count != 0){
                        // map_downsampled.data[i * map_downsampled.info.width + j] = 100;
                        map_downsampled.data.push_back(100);
                    } else {
                        if (free_count > unk_count){
                            // map_downsampled.data[i * map_downsampled.info.width + j] = 0;
                            map_downsampled.data.push_back(0);
                        } else {
                            // map_downsampled.data[i * map_downsampled.info.width + j] = -1;
                            map_downsampled.data.push_back(-1);
                        }
                    }
                }
            }
            map_out_pub.publish(map_downsampled);
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "map_downsample");
    ros::NodeHandle nh;
    double res;
    if(!nh.getParam("map_downsample/resolution",res)){ROS_ERROR("Failed to get param 'resolution'"); return -1;}
    NodeLogic nl(nh,res);

    ros::spin();

    return 0;
}