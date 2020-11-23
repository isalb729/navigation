#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
namespace cm = costmap_2d;

namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace global_planner {

class PlannerWithCostmap : public GlobalPlanner {
    public:
        PlannerWithCostmap(string name, Costmap2DROS* cmap);
        bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp);

    private:
        void poseCallback(const rm::PoseStamped::ConstPtr& goal);
        Costmap2DROS* cmap_;
        ros::ServiceServer make_plan_service_;
        ros::Subscriber pose_sub_;
};

// find the path
bool PlannerWithCostmap::makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp) {
    
    vector<PoseStamped> path;
    req.start.header.frame_id = "map";
    req.goal.header.frame_id = "map";
    // makeplan is a method inherited from GlobalPlanner
    bool success = makePlan(req.start, req.goal, path);
    resp.plan_found = success;
    if (success) {
        resp.path = path;
    }
    return true;
}

void PlannerWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
    geometry_msgs::PoseStamped global_pose;
    // transfer the pose to global frame
    cmap_->getRobotPose(global_pose);
    vector<PoseStamped> path;
    // the plan would be published!
    makePlan(global_pose, *goal, path);
}

PlannerWithCostmap::PlannerWithCostmap(string name, Costmap2DROS* cmap) :
        GlobalPlanner(name, cmap->getCostmap(), cmap->getGlobalFrameID()) {
    ros::NodeHandle private_nh("~");
    cmap_ = cmap;
    make_plan_service_ = private_nh.advertiseService("make_plan", &PlannerWithCostmap::makePlanService, this);
    pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &PlannerWithCostmap::poseCallback, this);
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS lcr("costmap", buffer);

    global_planner::PlannerWithCostmap pppp("planner", &lcr);

    ros::spin();
    return 0;
}
