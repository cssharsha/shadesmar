#pragma once

#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "utils/stf/proto/transform_tree.pb.h"

namespace stf {

class TransformTree {
public:
    using Transform = Eigen::Isometry3d;

    struct Edge {
        std::string parent;
        std::string child;
        Transform transform;

        Edge(const std::string& p, const std::string& c, const Transform& t)
            : parent(p), child(c), transform(t) {}
    };

    struct Node {
        std::string frame_id;
        std::unordered_map<std::string, std::shared_ptr<Edge>> children;
        std::unordered_map<std::string, std::shared_ptr<Edge>> parents;

        explicit Node(const std::string& id) : frame_id(id) {}
    };

    struct TransformResult {
        Transform transform;
        std::string path;
    };

    /**
     * Sets or updates a transform between parent and child frames
     * @param parent_frame Parent frame ID
     * @param child_frame Child frame ID
     * @param transform Transform from parent to child
     * @throws std::runtime_error if adding this transform would create a cycle
     */
    void setTransform(const std::string& parent_frame, const std::string& child_frame,
                      const Transform& transform);

    /**
     * Gets transform between any two frames in the tree
     * @param parent_frame Source frame ID
     * @param child_frame Target frame ID
     * @return Transform from parent to child
     * @throws std::runtime_error if frames are not connected
     */
    TransformResult getTransform(const std::string& parent_frame,
                                 const std::string& child_frame) const;

    /**
     * Prints a visual representation of the transform tree
     * Shows the hierarchy of frames and their connections
     */
    void printTree() const;

    /**
     * Serializes the transform tree to protobuf format
     * @param tree_proto Output protobuf message
     * @return true if serialization was successful
     */
    bool serializeToProto(stf::proto::TransformTreeSnapshot& tree_proto) const;

    /**
     * Deserializes the transform tree from protobuf format
     * @param tree_proto Input protobuf message
     * @return true if deserialization was successful
     */
    bool deserializeFromProto(const stf::proto::TransformTreeSnapshot& tree_proto);

    /**
     * Saves the transform tree to a file
     * @param filepath File path to save to
     * @return true if save was successful
     */
    bool saveToFile(const std::string& filepath) const;

    /**
     * Loads the transform tree from a file
     * @param filepath File path to load from
     * @return true if load was successful
     */
    bool loadFromFile(const std::string& filepath);

private:
    std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;

    /**
     * Finds a path between two frames using BFS
     * @param start Start frame ID
     * @param end End frame ID
     * @param path Path found
     * @return true if a path is found, false otherwise
     */
    bool findPath(const std::string& start, const std::string& end,
                  std::vector<std::shared_ptr<Edge>>& path) const;

    /**
     * Helper function for finding a path between two frames using DFS
     * @param current Current frame being visited
     * @param target Target frame to find
     * @param visited Set of visited frames
     * @param current_path Current path being built
     * @param final_path Final path found
     * @return true if a path is found, false otherwise
     */
    bool findPathDFS(const std::string& current, const std::string& target,
                     std::unordered_set<std::string>& visited,
                     std::vector<std::shared_ptr<Edge>>& current_path,
                     std::vector<std::shared_ptr<Edge>>& final_path) const;

    Transform computeTransform(const std::vector<std::shared_ptr<Edge>>& path) const;

    /**
     * Checks if adding an edge would create a cycle in the graph
     * @param parent_frame Parent frame ID
     * @param child_frame Child frame ID
     * @return true if adding the edge would create a cycle
     */
    bool wouldCreateCycle(const std::string& parent_frame, const std::string& child_frame) const;

    /**
     * Helper function for cycle detection using DFS
     * @param current Current frame being visited
     * @param target Target frame to find
     * @param visited Set of visited frames
     * @return true if a path exists from current to target
     */
    bool hasPath(const std::string& current, const std::string& target,
                 std::unordered_set<std::string>& visited) const;

    /**
     * Helper function for printTree to recursively print the tree structure
     * @param frame_id Current frame being printed
     * @param visited Set of visited frames
     * @param indent Current indentation level
     * @param is_last Whether this is the last child in current level
     */
    void printTreeRecursive(const std::string& frame_id, std::unordered_set<std::string>& visited,
                            const std::string& indent, bool is_last) const;
};

}  // namespace stf
