#include "planner/search.h"

using namespace boost::property_tree;

namespace truck::planner::search {

Grid::Grid(const std::vector<Node>& nodes) { nodes_ = nodes; }

GridBuilder::GridBuilder(const GridParams& params) { params_ = params; }

GridBuilder& GridBuilder::setEgoPose(const geom::Pose& ego_pose) {
    ego_pose_ = ego_pose;
    return *this;
}

GridBuilder& GridBuilder::setFinishArea(const geom::Circle& finish_area) {
    finish_area_ = finish_area;
    return *this;
}

GridBuilder& GridBuilder::setCollisionChecker(
    const std::shared_ptr<collision::StaticCollisionChecker> checker) {
    checker_ = checker;
    return *this;
}

bool GridBuilder::insideFinishArea(const geom::Vec2& point) {
    return (finish_area_.value().center - point).lenSq() < squared(finish_area_.value().radius);
}

Grid GridBuilder::build() {
    std::vector<Node> nodes;

    // determining origin point - lower left node of the grid
    geom::Vec2 origin_point(
        ego_pose_.value().pos.x - (params_.width / 2),
        ego_pose_.value().pos.y - (params_.height / 2));

    // to prevent the grid from jumping in the odometric world,
    // origin point can only have coordinates with constraint (i*resolution, j*resoultion)
    origin_point = geom::Vec2(
        round<double>(origin_point.x / params_.resolution) * params_.resolution,
        round<double>(origin_point.y / params_.resolution) * params_.resolution);

    // initializaing nodes
    for (int i = 0; i < int(params_.height / params_.resolution); i++) {
        for (int j = 0; j < int(params_.width / params_.resolution); j++) {
            Node node;
            node.id = NodeId{j, i};
            node.point = origin_point + (geom::Vec2(j, i) * params_.resolution);

            node.is_finish = insideFinishArea(node.point);
            node.collision = (checker_->distance(node.point) == 0.0f) ? true : false;

            /** @todo find start node (bool or pointer) */

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

    for (ptree::iterator it_primitive = primitives_tree.begin();
         it_primitive != primitives_tree.end();
         it_primitive++) {
        Primitive primitive;

        ptree points_tree = it_primitive->second;

        for (ptree::iterator it_point = points_tree.begin(); it_point != points_tree.end();
             it_point++) {
            // read data
            double x = atof(it_point->second.get_child("x").data().c_str());
            double y = atof(it_point->second.get_child("y").data().c_str());
            double yaw = atof(it_point->second.get_child("yaw").data().c_str());

            // initialize point
            geom::Pose pose(geom::Vec2(x, y), geom::Vec2(geom::Angle(yaw)));

            // add point to the current primitive
            primitive.push_back(pose);
        }

        // add primitive to the list
        primitives_.push_back(primitive);
    }
}

Vertex::Vertex(
    NodeId node_id, size_t yaw_index,
    const std::shared_ptr<EdgeGeometryCache> edge_geometry_cache) {
    this->node_id = node_id;
    this->yaw_index = yaw_index;

    for (const Primitive& primitive : edge_geometry_cache->primitives_) {
        /** @todo add pointers to 'edges' array */
    }
}

DynamicGraph::DynamicGraph() {}

DynamicGraph& DynamicGraph::setGrid(const std::shared_ptr<Grid> grid) {
    grid_ = grid;
    return *this;
}

DynamicGraph& DynamicGraph::setYawBins(const std::shared_ptr<YawBins> yaw_bins) {
    yaw_bins_ = yaw_bins;
    return *this;
}

DynamicGraph& DynamicGraph::setEdgeGeometryCache(
    const std::shared_ptr<EdgeGeometryCache> edge_geometry_cache) {
    edge_geometry_cache_ = edge_geometry_cache;
    return *this;
}

Searcher::Searcher(const std::shared_ptr<DynamicGraph> graph) { graph_ = graph; }

void Searcher::findPath() { /** @todo */
}

}  // namespace truck::planner::search