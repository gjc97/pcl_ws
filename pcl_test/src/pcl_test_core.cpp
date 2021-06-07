#include "pcl_test_core.h"
//PclTestCore core(nh);
PclTestCore::PclTestCore(ros::NodeHandle &nh)
{
    //创建一个发布者，订阅/points_raw话题，通过this指针来调用类内的回调函数。
    sub_point_cloud_ = nh.subscribe("/points_raw", 10, &PclTestCore::point_cb, this);
    //发布过分割出来的地面点云
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);
    //发布滤除地面之后的点
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);
    //发布voxel_grid_filter之后的点
    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
    ros::spin();
}

PclTestCore::~PclTestCore() {}

void PclTestCore::Spin() {}
//参数列表：高度；输入点云（pcl格式点云）；输出点云
void PclTestCore::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    //创建滤波对象
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

void PclTestCore::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void PclTestCore::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   PointCloudXYZIRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    // std::cout<<"输出的排序后点云size为"<<std::endl;
    // ROS_WARN_STREAM(out_organized_points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZIRTColor new_point;
        //点云在xy平面距离圆心的距离
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
            //与x轴正方向的夹角单位度
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分（索引？）
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        //radial divisions更加角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);
        //安照径向重排序后的点云
        out_radial_ordered_clouds[radial_div].push_back(new_point);

    } //end for

    //将同一根射线上的点按照半径（距离）排序
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
//通过判断同一条射线前后两点之间的坡度是否大于事先设定的坡度阈值，判断点是否为地面点
/*判断原理：通过8和5两个坡度阈值结合此点的半径，计算出高度阈值，通过判断当前点的z值是否在高度阈值的加减范围内
判断是否为地面点*/
void PclTestCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线，判断是否为地面
    {   //测试输出size为2250
        //ROS_WARN_STREAM(in_radial_ordered_clouds.size());
        float prev_radius = 0.f;
        //前一个点的高度：初始化传感器高度的负数，即地面的高度
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div按照径向遍历每一个点
        {
            //测试输出size.大概为16，24，25，23
            //ROS_WARN_STREAM(in_radial_ordered_clouds[i].size());
            //某个点的距离=当前点的半径-前一个点的半径（同一条射线）；
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            //DEG2RAD：角度转弧度;height_threshold就是在当前坡度阈值下的最大高度。
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            //i从0-2249，j的范围不确定
            //current_height：每个点的真实的高度，激光雷达以下应该为负数
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            //根据半径求出的高度：当前地面最大坡度阈值下的最大高度
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

            //for points which are very close causing the height threshold to be tiny, set a minimum value
            //对于距离非常近导致高度阈值很小的点，为其设置一个最小值
            //如果距离大于最小距离0.01且高度阈值小于最小值，则高度阈值等于最小值
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            //check current point height against the LOCAL threshold (previous point)
            //对照llocal阈值检查当前点的高度，
            //检查current_height是否属于区间[prev_height-height_threshold,prev_height+height_threshold]
            //height_threshold是使用最大角度8计算出的数值
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                //如果不是地面点，使用半径再次筛选

                if (!prev_ground)//如果前一个点不是地面
                {
                    //general_height_threshold是使用常规角度5计算出来的数值
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    //在8度的坡度范围内，但是没有在5度的坡度范围内，即：去除由于地面自带的轻微坡度带来的影响
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            //使用8的坡度没有过滤掉点
            else
            {
                //check if previous point is too far from previous one, if so classify again
                //检查上一个点是否距离上一个点太远，是的话重新分类
                //如果当前点在8度坡度范围内并且当前点相对于前一个点的距离大于0.2，则也认为是地面点
                if (points_distance > reclass_distance_threshold_ &&
                (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }
            //如果当前点被认为是地面点，则将此点的原始索引存储到地面点索引中，并将prev_ground置为true
            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }

            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }
            //将当前点作为前一个点
            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}
//函数参数含义：1：发布者的名称；2：需要作为消息进行发布的pcl类型的点云指针；3：消息头
void PclTestCore::publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    //创建指向<pcl::PointXYZI>类型的pcl::PointCloud类的空（智能？）指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);//当前点云，用来存储sensor类型转换为pcl类型的数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);//切割后的点云

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    //将高于雷达0.2米的点云去除
    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);
    //去除距离雷达<2.4m的点
    remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);

    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

    //将点云按照segment分割为2250份
    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);
    //将点云进行排序？（容易根线上的点云按照半径从大到小排列
    XYZI_to_RTZColor(remove_close, organized_points,
                     radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;
    //对点云进行分类：地面点和非地面点
    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);//添加filter_pc_ptr指针，用来接受filter之后的点云数据
    //pcl::ExtractIndices（滤波器对象）利用索引提取点云子集
    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    //输入参数：移除距离小于2.4m（距离车子太近）后的点
    extract_ground.setInputCloud(remove_close);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //参数为false则留下索引true removes the indices, false leaves only the indices，true移除索引
    //将留下地面索引的点云赋值给ground_cloud_ptr指针
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //参数为true则删除索引true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*filtered_pc_ptr);
    publish_cloud(pub_filtered_points_, filtered_pc_ptr, in_cloud_ptr->header);
    //3：消息头
    publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    
    publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);

    //////pub for debug
    // sensor_msgs::PointCloud2 pub_pc;
    // pcl::toROSMsg(*remove_close, pub_pc);

    // pub_pc.header = in_cloud_ptr->header;

    // pub_ground_.publish(pub_pc);
    //publish_cloud函数参数含义：1：传入的publisher；2：要发布的消息类型 的指针

}