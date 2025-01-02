#include "stf/transform_tree.hpp"
#include <queue>
#include <unordered_set>

namespace stf {

void TransformTree::setTransform(const std::string& parent_frame,
                               const std::string& child_frame,
                               const Transform& transform) {
    if (parent_frame == child_frame) {
        throw std::runtime_error("Cannot create transform from frame to itself: " + parent_frame);
    }

    // Check for cycles before modifying the graph
    if (wouldCreateCycle(parent_frame, child_frame)) {
        throw std::runtime_error("Adding transform from " + parent_frame +
                               " to " + child_frame + " would create a cycle");
    }

    // Create nodes if they don't exist
    if (nodes_.find(parent_frame) == nodes_.end()) {
        nodes_[parent_frame] = std::make_shared<Node>(parent_frame);
    }
    if (nodes_.find(child_frame) == nodes_.end()) {
        nodes_[child_frame] = std::make_shared<Node>(child_frame);
    }

    // Create or update the edge
    auto edge = std::make_shared<Edge>(parent_frame, child_frame, transform);
    nodes_[parent_frame]->children[child_frame] = edge;
    nodes_[child_frame]->parents[parent_frame] = edge;
}

bool TransformTree::wouldCreateCycle(const std::string& parent_frame,
                                   const std::string& child_frame) const {
    // If either frame doesn't exist yet, no cycle is possible
    if (nodes_.find(parent_frame) == nodes_.end() ||
        nodes_.find(child_frame) == nodes_.end()) {
        return false;
    }

    // Check if this is an update to an existing transform
    auto parent_node = nodes_.at(parent_frame);
    if (parent_node->children.find(child_frame) != parent_node->children.end()) {
        return false;  // Updating existing transform, no new cycle possible
    }

    // Check if there's already a path from child to parent
    std::unordered_set<std::string> visited;
    return hasPath(child_frame, parent_frame, visited);
}

bool TransformTree::hasPath(const std::string& current,
                          const std::string& target,
                          std::unordered_set<std::string>& visited) const {
    if (current == target) {
        return true;
    }

    visited.insert(current);
    auto current_node = nodes_.at(current);

    // Check children
    for (const auto& child : current_node->children) {
        if (visited.find(child.first) == visited.end()) {
            if (hasPath(child.first, target, visited)) {
                return true;
            }
        }
    }

    // Check parents
    for (const auto& parent : current_node->parents) {
        if (visited.find(parent.first) == visited.end()) {
            if (hasPath(parent.first, target, visited)) {
                return true;
            }
        }
    }

    return false;
}

TransformTree::TransformResult TransformTree::getTransform(const std::string& parent_frame,
                                    const std::string& child_frame) const {
    std::string path = parent_frame;

    // Handle identity case first
    if (parent_frame == child_frame) {
        return {Transform::Identity(), path};
    }

    // Find path between frames
    std::vector<std::shared_ptr<Edge>> edge_path;
    if (!findPath(parent_frame, child_frame, edge_path)) {
        throw std::runtime_error("No transform path exists between '" +
                               parent_frame + "' and '" + child_frame + "'");
    }

    // Build the path string
    for (const auto& edge : edge_path) {
        path += "/" + (edge->parent == path.substr(path.find_last_of('/') + 1)
            ? edge->child : edge->parent);
    }

    return {computeTransform(edge_path), path};
}

bool TransformTree::findPath(const std::string& start,
                           const std::string& end,
                           std::vector<std::shared_ptr<Edge>>& path) const {
    if (start == end) return true;

    std::unordered_set<std::string> visited;
    std::vector<std::shared_ptr<Edge>> current_path;

    return findPathDFS(start, end, visited, current_path, path);
}

bool TransformTree::findPathDFS(const std::string& current,
                              const std::string& target,
                              std::unordered_set<std::string>& visited,
                              std::vector<std::shared_ptr<Edge>>& current_path,
                              std::vector<std::shared_ptr<Edge>>& final_path) const {
    visited.insert(current);

    auto it = nodes_.find(current);
    if (it == nodes_.end()) return false;

    // Check both children and parents for connections
    for (const auto& [_, edge] : it->second->children) {
        if (visited.find(edge->child) != visited.end()) continue;

        current_path.push_back(edge);
        if (edge->child == target || findPathDFS(edge->child, target, visited, current_path, final_path)) {
            final_path = current_path;
            return true;
        }
        current_path.pop_back();
    }

    for (const auto& [_, edge] : it->second->parents) {
        if (visited.find(edge->parent) != visited.end()) continue;

        // Create a reversed edge for the path
        auto reversed_edge = std::make_shared<Edge>(edge->child, edge->parent, edge->transform.inverse());
        current_path.push_back(reversed_edge);
        if (edge->parent == target || findPathDFS(edge->parent, target, visited, current_path, final_path)) {
            final_path = current_path;
            return true;
        }
        current_path.pop_back();
    }

    return false;
}

TransformTree::Transform TransformTree::computeTransform(const std::vector<std::shared_ptr<Edge>>& path) const {
    Transform result = Transform::Identity();
    for (const auto& edge : path) {
        result = result * edge->transform;
    }
    return result;
}

void TransformTree::printTree() const {
    if (nodes_.empty()) {
        std::cout << "Empty transform tree" << std::endl;
        return;
    }

    // Find root nodes (nodes with no parents)
    std::vector<std::string> roots;
    for (const auto& [frame_id, node] : nodes_) {
        if (node->parents.empty()) {
            roots.push_back(frame_id);
        }
    }

    // If no root nodes found, just start with the first node
    if (roots.empty() && !nodes_.empty()) {
        roots.push_back(nodes_.begin()->first);
    }

    std::unordered_set<std::string> visited;
    for (size_t i = 0; i < roots.size(); ++i) {
        printTreeRecursive(roots[i], visited, "", i == roots.size() - 1);
    }
}

void TransformTree::printTreeRecursive(const std::string& frame_id,
                                     std::unordered_set<std::string>& visited,
                                     const std::string& indent,
                                     bool is_last) const {
    if (visited.find(frame_id) != visited.end()) {
        std::cout << indent << (is_last ? "└── " : "├── ") << frame_id << " (cycle)" << std::endl;
        return;
    }

    visited.insert(frame_id);

    // Print current node
    std::cout << indent << (is_last ? "└── " : "├── ") << frame_id << std::endl;

    // Get all connected frames (both children and parents)
    auto node = nodes_.at(frame_id);
    std::vector<std::pair<std::string, bool>> connections; // frame_id, is_child

    for (const auto& [child_id, _] : node->children) {
        connections.emplace_back(child_id, true);
    }
    for (const auto& [parent_id, _] : node->parents) {
        if (visited.find(parent_id) == visited.end()) {
            connections.emplace_back(parent_id, false);
        }
    }

    // Print all connected frames
    for (size_t i = 0; i < connections.size(); ++i) {
        const auto& [connected_frame, is_child] = connections[i];
        if (visited.find(connected_frame) != visited.end()) continue;

        std::string new_indent = indent + (is_last ? "    " : "│   ");
        printTreeRecursive(connected_frame,
                          visited,
                          new_indent,
                          i == connections.size() - 1);
    }
}

} // namespace stf
