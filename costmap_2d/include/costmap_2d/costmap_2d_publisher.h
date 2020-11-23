#ifndef COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#define COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

namespace costmap_2d
{
/**
 * @class Costmap2DPublisher
 * @brief A tool to periodically publish visualization data from a Costmap2D
 */
class Costmap2DPublisher
{
public:
  /**
   * @brief  Constructor for the Costmap2DPublisher
   */
  Costmap2DPublisher(ros::NodeHandle * ros_node, Costmap2D* costmap, std::string global_frame,
                     std::string topic_name, bool always_send_full_costmap = false);

  /**
   * @brief  Destructor
   */
  ~Costmap2DPublisher();

  /** @brief Include the given bounds in the changed-rectangle. */
  void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
  {
    x0_ = std::min(x0, x0_);
    xn_ = std::max(xn, xn_);
    y0_ = std::min(y0, y0_);
    yn_ = std::max(yn, yn_);
  }

  /**
   * @brief  Publishes the visualization data over ROS
   */
  void publishCostmap();

  /**
   * @brief Check if the publisher is active
   * @return True if the frequency for the publisher is non-zero, false otherwise
   */
  bool active()
  {
    return active_;
  }

private:
  /** @brief Prepare grid_ message for publication. */
  void prepareGrid();

  /** @brief Publish the latest full costmap to the new subscriber. */
  void onNewSubscription(const ros::SingleSubscriberPublisher& pub);

  ros::NodeHandle* node;
  Costmap2D* costmap_;
  std::string global_frame_;
  unsigned int x0_, xn_, y0_, yn_;
  double saved_origin_x_, saved_origin_y_;
  bool active_;
  bool always_send_full_costmap_;
  ros::Publisher costmap_pub_;
  ros::Publisher costmap_update_pub_;
  nav_msgs::OccupancyGrid grid_;
  static char* cost_translation_table_;  ///< Translate from 0-255 values in costmap to -1 to 100 values in message.
};
}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_2D_PUBLISHER_H
