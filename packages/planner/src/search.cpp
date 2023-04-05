#include "planner/search.h"

using namespace boost::property_tree;

namespace truck::planner::search {

Grid::Grid(std::vector<Node>& nodes) {
    nodes_ = nodes;
}

std::vector<Node> Grid::getNodes() {
    return nodes_;
}

GridBuilder::GridBuilder(const GridParams& params) {
    params_ = params;
}

GridBuilder& GridBuilder::setEgoPose(const geom::Pose& ego_pose) {
    ego_pose_ = ego_pose;
    return *this;
}

GridBuilder& GridBuilder::setFinishArea(const geom::Circle& finish_area) {
    finish_area_ = finish_area;
    return *this;
}

GridBuilder& GridBuilder::setCollisionChecker(std::shared_ptr<collision::StaticCollisionChecker> collision_checker) {
    collision_checker_ = collision_checker;
    return *this;
}

bool GridBuilder::insideFinishArea(const geom::Vec2& point) { 
    return (finish_area_.value().center - point).lenSq() < pow(finish_area_.value().radius, 2);
}

Grid GridBuilder::build() {
    std::vector<Node> nodes;

    geom::Vec2 origin_point(
        -(params_.width / 2) + ego_pose_.value().pos.x,
        -(params_.height / 2) + ego_pose_.value().pos.y
    );

    for (int i = 0; i < int(params_.height / params_.resolution); i++) {
        for (int j = 0; j < int(params_.width / params_.resolution); j++) {
            Node node;
            node.id = NodeId{j, i};
            node.point = origin_point + (geom::Vec2(j, i) * params_.resolution);
            
            node.is_finish = insideFinishArea(node.point);
            node.is_obstacle = (collision_checker_->distance(node.point) == 0.0f) ? true : false;
            
            nodes.push_back(node);
        }
    }

    return Grid(nodes);
}

YawBins::YawBins(const std::string& path) {
    std::ifstream data(path, std::ios_base::in);
    ptree main_tree, yaws_tree;
    read_json(data, main_tree);
    yaws_tree = main_tree.get_child("yaws");

    for (ptree::iterator it = yaws_tree.begin(); it != yaws_tree.end(); it++) {
        // add yaw to the list
        yaws_.push_back(atof(it->second.data().c_str()));
    }
}

EdgeGeometryCache::EdgeGeometryCache(const std::string& path) {
    std::ifstream data(path, std::ios_base::in);
    ptree main_tree, primitives_tree;
    read_json(data, main_tree);
    primitives_tree = main_tree.get_child("primitives");

    for (
        ptree::iterator it_primitive = primitives_tree.begin();
        it_primitive != primitives_tree.end();
        it_primitive++) {
        
        Primitive primitive;

        ptree points_tree = it_primitive->second;

        for (
            ptree::iterator it_point = points_tree.begin();
            it_point != points_tree.end();
            it_point++) {
            
            // read data
            double x = atof(it_point->second.get_child("x").data().c_str());
            double y = atof(it_point->second.get_child("y").data().c_str());
            double yaw = atof(it_point->second.get_child("yaw").data().c_str());

            // yaws_.push_back(atof(it->second.data().c_str()));

            // initialize point
            geom::Pose pose(
                geom::Vec2(x, y),
                geom::Vec2(geom::Angle(yaw))
            );

            // add point to the current primitive
            primitive.push_back(pose);
        }

        // add primitive to the list
        primitives_.push_back(primitive);
    }
}

DynamicGraph::DynamicGraph() {}

DynamicGraph& DynamicGraph::setGrid(std::shared_ptr<Grid> grid) {
    grid_ = grid;
    return *this;
}

DynamicGraph& DynamicGraph::setYawBins(std::shared_ptr<YawBins> yaw_bins) {
    yaw_bins_ = yaw_bins;
    return *this;
}

DynamicGraph& DynamicGraph::setEdgeGeometryCache(std::shared_ptr<EdgeGeometryCache> edge_geometry_cache) {
    edge_geometry_cache_ = edge_geometry_cache;
    return *this;
}

DynamicGraph& DynamicGraph::setCollisionChecker(std::shared_ptr<collision::StaticCollisionChecker> collision_checker) {
    collision_checker_ = collision_checker;
    return *this;
}

Searcher::Searcher(std::shared_ptr<DynamicGraph> graph) {
    graph_ = graph;
}

void Searcher::findPath() { /** @todo */ }

std::vector<geom::Pose> Searcher::getFoundPath() {
    return path_;
}

} // namespace truck::planner::search