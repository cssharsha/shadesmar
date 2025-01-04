#include "core/graph/graph_adapter.hpp"
#include "core/graph/util.hpp"
#include "logging/logging.hpp"

namespace core {
namespace graph {

namespace {
double calculateDistance(const types::Pose& relative_pose) {
    return relative_pose.position.norm();
}
}  // namespace

void GraphAdapter::handleLoopClosure(uint64_t from_id, uint64_t to_id,
                                     const types::Pose& relative_pose) {
    types::Factor loop_factor;
    loop_factor.type = proto::FactorType::LOOP_CLOSURE;
    loop_factor.connected_nodes = {from_id, to_id};
    loop_factor.measurement.emplace<0>(relative_pose);
    loop_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

    graph_.addFactor(loop_factor);
    store_.addFactor(loop_factor);

    // Loop closures don't count towards cumulative distance
    maybeDumpGraph();
}

void GraphAdapter::addOdometryFactor(uint64_t from_id, uint64_t to_id,
                                     const types::Pose& relative_pose) {
    types::Factor odom_factor;
    odom_factor.type = proto::FactorType::ODOMETRY;
    odom_factor.connected_nodes = {from_id, to_id};
    odom_factor.measurement.emplace<0>(relative_pose);
    odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

    graph_.addFactor(odom_factor);
    store_.addFactor(odom_factor);

    // Update cumulative distance and maybe dump graph
    cumulative_distance_ += calculateDistance(relative_pose);
    maybeDumpGraph();
}

void GraphAdapter::maybeDumpGraph() {
    constexpr double DUMP_INTERVAL = 1.0;  // meters

    if (cumulative_distance_ >= next_dump_distance_) {
        LOG(INFO) << "Dumping factor graph at distance " << cumulative_distance_ << "m";

        // Create filename with distance
        std::string filename = "/mnt/remote-storage/factor_graph_" +
                               std::to_string(static_cast<int>(cumulative_distance_)) + "m.vtk";

        util::dumpFactorGraph(graph_, filename);
        next_dump_distance_ = cumulative_distance_ + DUMP_INTERVAL;

        LOG(INFO) << "Next graph dump at " << next_dump_distance_ << "m";
    }
}

}  // namespace graph
}  // namespace core
