// include/knowrob_ros/ROSInterface.hpp

#ifndef KNOWROB_ROS__ROS_INTERFACE_HPP_
#define KNOWROB_ROS__ROS_INTERFACE_HPP_

// C++ std
#include <map>
#include <mutex>
#include <unordered_map>
#include <memory>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/any.hpp>

// rclcpp & rclcpp_action
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// KnowRob core
#include <knowrob/knowrob.h>
#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include <knowrob/queries/QueryParser.h>
#include <knowrob/formulas/ModalFormula.h>
#include <knowrob/terms/String.h>

// Generated interfaces
#include "knowrob_ros/msg/graph_query_message.hpp"
#include "knowrob_ros/msg/graph_answer_message.hpp"
#include "knowrob_ros/msg/key_value_pair.hpp"
#include "knowrob_ros/msg/modal_frame.hpp"

#include "knowrob_ros/action/ask_all.hpp"
#include "knowrob_ros/action/ask_one.hpp"
#include "knowrob_ros/action/ask_incremental.hpp"
#include "knowrob_ros/action/ask_incremental_next_solution.hpp"
#include "knowrob_ros/action/tell.hpp"

#include "knowrob_ros/srv/ask_incremental_finish.hpp"
#include "knowrob_ros/srv/export_triples.hpp"

namespace knowrob_ros
{

class ROSInterface : public rclcpp::Node
{
public:
  explicit ROSInterface(const boost::property_tree::ptree & ptree);
  ~ROSInterface() override = default;

private:
  // Type aliases for readability
  using AskAll             = action::AskAll;
  using AskOne             = action::AskOne;
  using AskIncremental     = action::AskIncremental;
  using AskIncrementalNext = action::AskIncrementalNextSolution;
  using Tell               = action::Tell;

  using GoalHandleAskAll         = rclcpp_action::ServerGoalHandle<AskAll>;
  using GoalHandleAskOne         = rclcpp_action::ServerGoalHandle<AskOne>;
  using GoalHandleAskIncremental = rclcpp_action::ServerGoalHandle<AskIncremental>;
  using GoalHandleAskIncrementalNext
                              = rclcpp_action::ServerGoalHandle<AskIncrementalNext>;
  using GoalHandleTell           = rclcpp_action::ServerGoalHandle<Tell>;

  // Helpers to bridge KnowRob types ↔ ROS messages
  std::unordered_map<std::string, boost::any>
    translateModalityFrameMessage(const msg::ModalFrame & frame);

  msg::GraphAnswerMessage
    createGraphAnswer(std::shared_ptr<const knowrob::AnswerYes> answer);

  // —— Action servers —————————————————————————————
  std::shared_ptr<rclcpp_action::Server<AskAll>>             askall_action_server_;
  std::shared_ptr<rclcpp_action::Server<AskOne>>             askone_action_server_;
  std::shared_ptr<rclcpp_action::Server<AskIncremental>>     askincremental_action_server_;
  std::shared_ptr<rclcpp_action::Server<AskIncrementalNext>> askincremental_next_action_server_;
  std::shared_ptr<rclcpp_action::Server<Tell>>               tell_action_server_;

  // —— Services ——————————————————————————————————————
  rclcpp::Service<srv::AskIncrementalFinish>::SharedPtr ask_incremental_finish_srv_;
  rclcpp::Service<srv::ExportTriples>::SharedPtr        export_srv_;

  // —— Query state ————————————————————————————————
  std::mutex                                         query_mutex_;
  std::map<uint32_t, knowrob::QueryResultQueuePtr>  query_results_;
  uint32_t                                           next_query_id_ = 1;

  // —— KnowRob knowledge base —————————————————————
  knowrob::KnowledgeBasePtr                         kb_;

  // —— Callbacks for each action & service ——————
  // (signatures match the implementations in your .cpp)
  rclcpp_action::GoalResponse  handle_goal_askall(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const AskAll::Goal>);
  rclcpp_action::CancelResponse handle_cancel_askall(
                                const std::shared_ptr<GoalHandleAskAll>);
  void                          handle_accepted_askall(
                                const std::shared_ptr<GoalHandleAskAll>);

  rclcpp_action::GoalResponse  handle_goal_askone(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const AskOne::Goal>);
  rclcpp_action::CancelResponse handle_cancel_askone(
                                const std::shared_ptr<GoalHandleAskOne>);
  void                          handle_accepted_askone(
                                const std::shared_ptr<GoalHandleAskOne>);

  rclcpp_action::GoalResponse  handle_goal_askincremental(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const AskIncremental::Goal>);
  rclcpp_action::CancelResponse handle_cancel_askincremental(
                                const std::shared_ptr<GoalHandleAskIncremental>);
  void                          handle_accepted_askincremental(
                                const std::shared_ptr<GoalHandleAskIncremental>);

  rclcpp_action::GoalResponse  handle_goal_askincremental_next(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const AskIncrementalNext::Goal>);
  rclcpp_action::CancelResponse handle_cancel_askincremental_next(
                                const std::shared_ptr<GoalHandleAskIncrementalNext>);
  void                          handle_accepted_askincremental_next(
                                const std::shared_ptr<GoalHandleAskIncrementalNext>);

  rclcpp_action::GoalResponse  handle_goal_tell(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const Tell::Goal>);
  rclcpp_action::CancelResponse handle_cancel_tell(
                                const std::shared_ptr<GoalHandleTell>);
  void                          handle_accepted_tell(
                                const std::shared_ptr<GoalHandleTell>);

  void handle_ask_incremental_finish(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::AskIncrementalFinish::Request>,
    std::shared_ptr<srv::AskIncrementalFinish::Response>);

  void handle_export_triples(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::ExportTriples::Request>,
    std::shared_ptr<srv::ExportTriples::Response>);
};

}  // namespace knowrob_ros

#endif  // KNOWROB_ROS__ROS_INTERFACE_HPP_
