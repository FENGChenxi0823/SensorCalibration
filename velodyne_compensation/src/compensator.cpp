#include "velodyne_compensation/compensator.h"

#include "ros/this_node.h"
#include <iomanip>

Compensator::Compensator(ros::NodeHandle& node, ros::NodeHandle& private_nh)
    : x_offset_(-1),
      y_offset_(-1),
      z_offset_(-1),
      timestamp_offset_(-1),
      timestamp_data_size_(0) {
  // private_nh.param("child_frame_id", child_frame_id_,
  //                  std::string("velodyne64"));
  private_nh.param("topic_compensated_pointcloud",
                   topic_compensated_pointcloud_, topic_compensated_pointcloud_);
  private_nh.param("topic_pointcloud", topic_pointcloud_, topic_pointcloud_);
  private_nh.param("topic_transform", topic_transform_, topic_transform_);
  private_nh.param("queue_size", queue_size_, 100);
  // private_nh.param("translation",translation, translation);
  // private_nh.param("rotation",rotation, rotation);
  // private_nh.param("tf_query_timeout", tf_timeout_, float(0.1));
  // Eigen::Vector3d translation(1.7786, -1.25246, -0.72577);
  // Eigen::Quaterniond rotation(0.999841, 0.0143144, -0.0105069, -0.00185238);
  // advertise output point cloud (before subscribing to input data)

  compensation_pub_ = node.advertise<sensor_msgs::PointCloud2>(
      topic_compensated_pointcloud_, queue_size_);
  transform_sub_ = 
  	  node.subscribe(topic_transform_, queue_size_,
                     &Compensator::transformCallback, (Compensator*)this);
  pointcloud_sub_ =
      node.subscribe(topic_pointcloud_, queue_size_,
                     &Compensator::pointcloudCallback, (Compensator*)this);

}

void Compensator::transformCallback(const geometry_msgs::TransformStampedConstPtr& msg){
	std::pair <double, Transform> t;
    double stamp = msg->header.stamp.toSec();

    Transform T(Transform::Translation(msg->transform.translation.x,
                                       msg->transform.translation.y,
                                       msg->transform.translation.z),
                Transform::Rotation(msg->transform.rotation.w,
                                    msg->transform.rotation.x,
                                    msg->transform.rotation.y,
                                    msg->transform.rotation.z));
    t.first = stamp;
    t.second = T;
	trans_data_.push_back(t);
}

void Compensator::pointcloudCallback(sensor_msgs::PointCloud2ConstPtr msg){
	
	if (!checkMessage(msg)) {
    	ROS_FATAL("MotionCompensation : Input point cloud data field is invalid");
    	return;
  	}

  	double timestamp_min = 0.0;
  	double timestamp_max = 0.0;
  	getTimestampInterval(msg, timestamp_min, timestamp_max);
    double ideal_timestamp_max = timestamp_min + 0.1; //original timstamp is wrong, hardcoded by chenxi 

  	Eigen::Affine3d pose_min_time = getTransform(timestamp_min);
  	Eigen::Affine3d ideal_pose_max_time = getTransform(ideal_timestamp_max);

  	sensor_msgs::PointCloud2::Ptr q_msg(new sensor_msgs::PointCloud2());
  	*q_msg = *msg;
    motionCompensation(q_msg, timestamp_min, timestamp_max, pose_min_time, ideal_pose_max_time);
    // double *x = reinterpret_cast<double*>(&q_msg->data[0]);
    // double *y = reinterpret_cast<double*>(&q_msg->data[4]);
    // double *z = reinterpret_cast<double*>(&q_msg->data[8]);
    // std::cout << "data point x: " <<  *x << " y " << *y << " z " << *z << std::endl;
    q_msg->header.stamp.fromSec(timestamp_max);
    compensation_pub_.publish(q_msg);
    // trans_data_.erase(trans_data_.begin(), trans_data_.begin()+ buffer);;
}


inline void Compensator::getTimestampInterval(
    sensor_msgs::PointCloud2ConstPtr msg, double& timestamp_min,
    double& timestamp_max) {
  	timestamp_max = 0.0;
  	timestamp_min = std::numeric_limits<double>::max();
  	int total = msg->width * msg->height;

  	// get min time and max time
  	for (int i = 0; i < total; ++i) {
    	double timestamp = 0.0;
    	memcpy(&timestamp, &msg->data[i * msg->point_step + timestamp_offset_],
           timestamp_data_size_);

    	if (timestamp < timestamp_min) {
      		timestamp_min = timestamp;
    	}
    	if (timestamp > timestamp_max) {
      		timestamp_max = timestamp;
    	}
    	// std::cout << "timestamp: " << timestamp << std::endl;
  	}
}


inline bool Compensator::checkMessage(sensor_msgs::PointCloud2ConstPtr msg) {
  	// check msg width and height
  	if (msg->width == 0 || msg->height == 0) {
    	return false;
  	}

  	int x_data_type = 0;
  	int y_data_type = 0;
  	int z_data_type = 0;

  	// TODO: will use a new datastruct with interface to get offset,
  	// datatype,datasize...
  	for (size_t i = 0; i < msg->fields.size(); ++i) {
    	const sensor_msgs::PointField& f = msg->fields[i];

    	if (f.name == "x") {
      		x_offset_ = f.offset;
      		x_data_type = f.datatype;
      		if ((x_data_type != 7 && x_data_type != 8) || f.count != 1 || x_offset_ == -1) {
        		return false;
      		}
    	} 
    	else if (f.name == "y") {
      		y_offset_ = f.offset;
      		y_data_type = f.datatype;
      		if (f.count != 1 || y_offset_ == -1) {
        		return false;
      		}
    	} 
    	else if (f.name == "z") {
      		z_offset_ = f.offset;
      		z_data_type = f.datatype;
      		if (f.count != 1 || z_offset_ == -1) {
        	return false;
      		}
    	} 
    	else if (f.name == "timestamp") {
      		timestamp_offset_ = f.offset;
      		timestamp_data_size_ = f.count * getFieldSize(f.datatype);
      		// std::cout << "timestamp_data_size_" << timestamp_data_size_ <<std::endl;
      		if (timestamp_offset_ == -1 || timestamp_data_size_ == -1) {
        		return false;
      		}
    	} 
    	else {
      		ROS_DEBUG_STREAM("get a unused field name:" << f.name);
    	}
  	}

  // check offset if valid
  	if (x_offset_ == -1 || y_offset_ == -1 || z_offset_ == -1 ||
      	timestamp_offset_ == -1 || timestamp_data_size_ == -1) {
    	return false;
  	}
  	if (!(x_data_type == y_data_type && y_data_type == z_data_type)) {
    	return false;
  	}
  	return true;
}

inline uint Compensator::getFieldSize(const int datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;

    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;

    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;

    case sensor_msgs::PointField::FLOAT64:
      return 8;

    default:
      ROS_ERROR_STREAM("can not get field size by datatype:" << datatype);
      return 0;
  }
}


Eigen::Affine3d Compensator::getTransform(const double timestamp){
	int idx = 0;
	while(timestamp > trans_data_[idx].first 
		  && idx < (trans_data_.size()-1)){
		idx++;
	}
	if(idx > 0) --idx;

	double t_diff_ratio =
      static_cast<double>(timestamp - trans_data_[idx].first) /
      static_cast<double>(trans_data_[idx + 1].first - trans_data_[idx].first);
    Transform::Vector6 diff_vector =
      (trans_data_[idx].second.inverse() * trans_data_[idx + 1].second).log();
  	Transform t =
      trans_data_[idx].second * Transform::exp(t_diff_ratio * diff_vector);
 	
    Eigen::Affine3d pose;
    // pose.matrix() = (t * T_ltoi_).matrix();
    pose.matrix() = t.matrix().cast<double>();


    // std::cout << "trans_data: " <<trans_data_.size() << " index: " << idx << std::endl;
    // std::cout << std::setprecision(15) << "corresponding transformation time : " << trans_data_[idx+1].first << " pc timestamp: " << timestamp << std::endl;
    // std::cout << "time_diff_ratio: " << t_diff_ratio <<std::endl;
    // std::cout << "pose \n" << pose.matrix() << std::endl;

    return pose;
}

// template <typename Scalar>
void Compensator::motionCompensation(sensor_msgs::PointCloud2::Ptr& msg,
									  const double timestamp_min,
                                      const double timestamp_max,
                                      const Eigen::Affine3d& pose_min_time,
                                      const Eigen::Affine3d& pose_max_time){
	using std::abs;
  	using std::sin;
  	using std::acos;

	Eigen::Vector3d translation = pose_min_time.translation() - pose_max_time.translation();
	Eigen::Quaterniond q_max(pose_max_time.linear());
	Eigen::Quaterniond q_min(pose_min_time.linear());
	Eigen::Quaterniond q1(q_max.conjugate() * q_min);
	Eigen::Quaterniond q0(Eigen::Quaterniond::Identity());
	q1.normalize();
	translation = q_max.conjugate() * translation;

	int total = msg->width * msg->height;

	double d = q0.dot(q1);
	double abs_d = abs(d);
	double f = 1.0 / (timestamp_max - timestamp_min);

	  // Threshold for a "significant" rotation from min_time to max_time:
	  // The LiDAR range accuracy is ~2 cm. Over 70 meters range, it means an angle
	  // of 0.02 / 70 = 0.0003 rad. So, we consider a rotation "significant" only if
	  // the scalar part of quaternion is less than cos(0.0003 / 2) = 1 - 1e-8.
	 const double theta = acos(abs_d);
	 const double sin_theta = sin(theta);
	 const double c1_sign = (d > 0) ? 1 : -1;

	 // std::cout << "point step : " << msg->point_step <<std::endl;
	  
	 for (int i = 0; i < total; ++i) {
	    size_t offset = i * msg->point_step;
	    
	    float* x_scalar =
	        reinterpret_cast<float*>(&msg->data[offset + x_offset_]);
	    if (std::isnan(*x_scalar)) {
	      ROS_DEBUG_STREAM("nan point do not need motion compensation");
	      continue;
	    }
	    float* y_scalar =
	        reinterpret_cast<float*>(&msg->data[offset + y_offset_]);
	    float* z_scalar =
	        reinterpret_cast<float*>(&msg->data[offset + z_offset_]);
	    Eigen::Vector3d p(*x_scalar, *y_scalar, *z_scalar);

	    // if(i == 1) std::cout<< "first point of initial scan:  x " 
	    // 	 << *(x_scalar) << " y "<< *(y_scalar) << " z " << *(z_scalar) << std::endl;
	    
	    double tp = 0.0;
	    memcpy(&tp, &msg->data[i * msg->point_step + timestamp_offset_],
	           timestamp_data_size_);
	    double t = (timestamp_max - tp) * f;
	    // if(i == 1) std::cout << "time_ratio: " << t <<std::endl;

	    Eigen::Translation3d ti(t * translation);

	    Eigen::Affine3d ti_affine(ti);

	    // if(i == 1) std::cout << "translation: \n" << ti_affine.matrix() << std::endl;

	    if (abs_d < 1.0 - 1.0e-8) {
	      // "significant". Do both rotation and translation.
	      double c0 = sin((1 - t) * theta) / sin_theta;
	      double c1 = sin(t * theta) / sin_theta * c1_sign;
	      Eigen::Quaterniond qi(c0 * q0.coeffs() + c1 * q1.coeffs());
	      Eigen::Affine3d trans = ti * qi;
	      p = trans * p;
	    } else {
	      // Not a "significant" rotation. Do translation only.
	      p = ti * p;
	      // if(i == 1) std::cout << "p: x " << p.x() << " y " <<p.y() <<" z " <<p.z() <<std::endl;
	    }
		    *x_scalar = p.x();
		    *y_scalar = p.y();
		    *z_scalar = p.z();
	    // if(i == 1) std::cout<< "compensated point of transformed scan:  x " 
	    // 	 << *x_scalar << " y "<< *y_scalar << " z " << *z_scalar
	    // 	 << "\n ----------------------------------------------------------------------------\n";
	  }
	  return;

}

