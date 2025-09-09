#include "knowrob_ros/ROSInterface.hpp"
#include <knowrob/queries/QueryParser.h>
#include <knowrob/queries/QueryError.h>
#include <knowrob/formulas/ModalFormula.h>
#include <knowrob/terms/ListTerm.h>
#include <boost/any.hpp>

using namespace std::placeholders;
using namespace knowrob_ros;
using namespace knowrob;

ROSInterface::ROSInterface(const boost::property_tree::ptree & config)
: Node("knowrob_node")
, kb_(KnowledgeBase::create(config))
{
  // --- AskAll ---
  askall_action_server_ = rclcpp_action::create_server<AskAll>(
    this,
    "knowrob/askall",
    std::bind(&ROSInterface::handle_goal_askall, this, _1, _2),
    std::bind(&ROSInterface::handle_cancel_askall, this, _1),
    std::bind(&ROSInterface::handle_accepted_askall, this, _1));

  // --- AskOne ---
  askone_action_server_ = rclcpp_action::create_server<AskOne>(
    this,
    "knowrob/askone",
    std::bind(&ROSInterface::handle_goal_askone, this, _1, _2),
    std::bind(&ROSInterface::handle_cancel_askone, this, _1),
    std::bind(&ROSInterface::handle_accepted_askone, this, _1));

  // --- AskIncremental ---
  askincremental_action_server_ = rclcpp_action::create_server<AskIncremental>(
    this,
    "knowrob/askincremental",
    std::bind(&ROSInterface::handle_goal_askincremental, this, _1, _2),
    std::bind(&ROSInterface::handle_cancel_askincremental, this, _1),
    std::bind(&ROSInterface::handle_accepted_askincremental, this, _1));

  // --- AskIncrementalNextSolution ---
  askincremental_next_solution_action_server_ =
    rclcpp_action::create_server<AskIncrementalNext>(
      this,
      "knowrob/askincremental_next_solution",
      std::bind(&ROSInterface::handle_goal_askincremental_next, this, _1, _2),
      std::bind(&ROSInterface::handle_cancel_askincremental_next, this, _1),
      std::bind(&ROSInterface::handle_accepted_askincremental_next, this, _1));

  // --- Tell ---
  tell_action_server_ = rclcpp_action::create_server<Tell>(
    this,
    "knowrob/tell",
    std::bind(&ROSInterface::handle_goal_tell, this, _1, _2),
    std::bind(&ROSInterface::handle_cancel_tell, this, _1),
    std::bind(&ROSInterface::handle_accepted_tell, this, _1));

  // --- Services ---
  ask_incremental_finish_srv_ = this->create_service<srv::AskIncrementalFinish>(
    "knowrob/askincremental_finish",
    std::bind(&ROSInterface::handle_ask_incremental_finish, this, _1, _2, _3));

  export_srv_ = this->create_service<srv::ExportTriples>(
    "knowrob/export",
    std::bind(&ROSInterface::handle_export_triples, this, _1, _2, _3));

  RCLCPP_INFO(get_logger(), "[KnowRob] ROS2 interface ready.");
}

std::unordered_map<std::string, boost::any>
ROSInterface::translateModalityFrameMessage(
  const msg::ModalFrame & frame)
{
  std::unordered_map<std::string, boost::any> opts;
  opts["epistemicOperator"] = static_cast<int>(frame.epistemic_operator);
  opts["aboutAgentIRI"]     = frame.about_agent_iri;
  opts["confidence"]        = frame.confidence;
  opts["temporalOperator"]  = static_cast<int>(frame.temporal_operator);
  opts["minPastTimestamp"]  = frame.min_past_timestamp;
  opts["maxPastTimestamp"]  = frame.max_past_timestamp;
  return opts;
}

msg::GraphAnswerMessage
ROSInterface::createGraphAnswer(std::shared_ptr<const AnswerYes> answer)
{
  msg::GraphAnswerMessage out;
  for (auto & p : *answer->substitution()) {
    msg::KeyValuePair kv;
    kv.key = p.first;
    auto term = p.second.second;
    switch (term->termType()) {
      case TermType::ATOMIC: {
        auto a = std::static_pointer_cast<Atomic>(term);
        if (a->atomicType()==AtomicType::STRING ||
            a->atomicType()==AtomicType::ATOM)
        {
          kv.type         = msg::KeyValuePair::TYPE_STRING;
          kv.value_string = a->stringForm();
        } else {
          auto num = std::static_pointer_cast<Numeric>(a);
          switch (num->xsdType()) {
            case XSDType::FLOAT:
            case XSDType::DOUBLE:
              kv.type         = msg::KeyValuePair::TYPE_FLOAT;
              kv.value_float  = num->asDouble();
              break;
            default:
              kv.type        = msg::KeyValuePair::TYPE_INT;
              kv.value_int   = static_cast<int32_t>(num->asLong());
              break;
          }
        }
        break;
      }
      default:
        break;
    }
    out.substitution.push_back(kv);
  }
  return out;
}

// ----------------- AskAll -----------------
rclcpp_action::GoalResponse
ROSInterface::handle_goal_askall(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AskAll::Goal>)
{
  RCLCPP_INFO(get_logger(), "AskAll goal received");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ROSInterface::handle_cancel_askall(
  const std::shared_ptr<GoalHandleAskAll>)
{
  RCLCPP_INFO(get_logger(), "AskAll goal canceled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ROSInterface::handle_accepted_askall(
  const std::shared_ptr<GoalHandleAskAll> goal_handle)
{
  // spin off in its own thread
  std::thread(&ROSInterface::execute_askall, this, goal_handle).detach();
}

void
ROSInterface::execute_askall(
  const std::shared_ptr<GoalHandleAskAll> goal_handle)
{
  auto goal = goal_handle->get_goal();
  FormulaPtr phi = QueryParser::parse(goal->query.query_string);
  FormulaPtr mPhi = InterfaceUtils::applyModality(
    translateModalityFrameMessage(goal->query.frame), phi);

  auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
  auto stream = kb_->submitQuery(mPhi, ctx);
  auto queue  = stream->createQueue();

  auto feedback = std::make_shared<AskAll::Feedback>();
  auto result   = std::make_shared<AskAll::Result>();
  int count = 0;

  while (true) {
    auto tok = queue->pop_front();
    if (tok->indicatesEndOfEvaluation()) {
      break;
    } else if (tok->tokenType()==TokenType::ANSWER_TOKEN) {
      auto ans = std::static_pointer_cast<const AnswerYes>(tok);
      auto ga  = createGraphAnswer(ans);
      result->answers.push_back(ga);
      ++count;
      feedback->number_of_solutions = count;
      goal_handle->publish_feedback(*feedback);
    }
  }

  result->status = (count>0)
    ? AskAll::Result::TRUE
    : AskAll::Result::FALSE;

  goal_handle->succeed(*result);
}

// ----------------- AskOne -----------------
rclcpp_action::GoalResponse
ROSInterface::handle_goal_askone(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AskOne::Goal>)
{
  RCLCPP_INFO(get_logger(), "AskOne goal received");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ROSInterface::handle_cancel_askone(
  const std::shared_ptr<GoalHandleAskOne>)
{
  RCLCPP_INFO(get_logger(), "AskOne goal canceled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ROSInterface::handle_accepted_askone(
  const std::shared_ptr<GoalHandleAskOne> goal_handle)
{
  std::thread(&ROSInterface::execute_askone, this, goal_handle).detach();
}

void
ROSInterface::execute_askone(
  const std::shared_ptr<GoalHandleAskOne> goal_handle)
{
  auto goal = goal_handle->get_goal();
  FormulaPtr phi = QueryParser::parse(goal->query.query_string);
  FormulaPtr mPhi = InterfaceUtils::applyModality(
    translateModalityFrameMessage(goal->query.frame), phi);

  auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
  auto stream = kb_->submitQuery(mPhi, ctx);
  auto queue  = stream->createQueue();

  auto feedback = std::make_shared<AskOne::Feedback>();
  auto result   = std::make_shared<AskOne::Result>();

  auto tok = queue->pop_front();
  if (tok->tokenType()==TokenType::ANSWER_TOKEN) {
    auto ans = std::static_pointer_cast<const AnswerYes>(tok);
    result->status = AskOne::Result::TRUE;
    result->answer = createGraphAnswer(ans);
  } else {
    result->status = AskOne::Result::FALSE;
  }
  feedback->finished = true;
  goal_handle->publish_feedback(*feedback);
  goal_handle->succeed(*result);
}

// ----------------- AskIncremental -----------------
rclcpp_action::GoalResponse
ROSInterface::handle_goal_askincremental(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AskIncremental::Goal>)
{
  RCLCPP_INFO(get_logger(), "AskIncremental goal received");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ROSInterface::handle_cancel_askincremental(
  const std::shared_ptr<GoalHandleAskIncremental>)
{
  RCLCPP_INFO(get_logger(), "AskIncremental canceled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ROSInterface::handle_accepted_askincremental(
  const std::shared_ptr<GoalHandleAskIncremental> goal_handle)
{
  std::thread(&ROSInterface::execute_askincremental, this, goal_handle).detach();
}

void
ROSInterface::execute_askincremental(
  const std::shared_ptr<GoalHandleAskIncremental> goal_handle)
{
  auto goal = goal_handle->get_goal();
  FormulaPtr phi = QueryParser::parse(goal->query.query_string);
  FormulaPtr mPhi = InterfaceUtils::applyModality(
    translateModalityFrameMessage(goal->query.frame), phi);

  auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
  auto stream = kb_->submitQuery(mPhi, ctx);
  auto queue  = stream->createQueue();

  // store for next-solution calls
  static uint32_t next_id = 1;
  {
    std::lock_guard<std::mutex> lock(query_mutex_);
    query_results_[next_id] = queue;
  }

  auto feedback = std::make_shared<AskIncremental::Feedback>();
  auto result   = std::make_shared<AskIncremental::Result>();
  result->query_id = next_id++;
  result->status   = AskIncremental::Result::TRUE;
  feedback->finished = true;
  goal_handle->publish_feedback(*feedback);
  goal_handle->succeed(*result);
}

// ----------------- AskIncrementalNextSolution -----------------
rclcpp_action::GoalResponse
ROSInterface::handle_goal_askincremental_next(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AskIncrementalNext::Goal>)
{
  RCLCPP_INFO(get_logger(), "AskIncrementalNextSolution request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ROSInterface::handle_cancel_askincremental_next(
  const std::shared_ptr<GoalHandleAskIncrementalNext>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ROSInterface::handle_accepted_askincremental_next(
  const std::shared_ptr<GoalHandleAskIncrementalNext> goal_handle)
{
  std::thread(&ROSInterface::execute_askincremental_next, this, goal_handle).detach();
}

void
ROSInterface::execute_askincremental_next(
  const std::shared_ptr<GoalHandleAskIncrementalNext> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<AskIncrementalNext::Feedback>();
  auto result   = std::make_shared<AskIncrementalNext::Result>();

  std::shared_ptr<QueryResultQueue> queue;
  {
    std::lock_guard<std::mutex> lock(query_mutex_);
    auto it = query_results_.find(goal->query_id);
    if (it == query_results_.end()) {
      result->status = AskIncrementalNext::Result::INVALID_QUERY_ID;
      goal_handle->publish_feedback(*feedback);
      goal_handle->succeed(*result);
      return;
    }
    queue = it->second;
  }

  auto tok = queue->pop_front();
  if (tok->tokenType()==TokenType::ANSWER_TOKEN) {
    auto ans = std::static_pointer_cast<const AnswerYes>(tok);
    result->status = AskIncrementalNext::Result::TRUE;
    result->answer = createGraphAnswer(ans);
  } else {
    result->status = AskIncrementalNext::Result::FALSE;
    std::lock_guard<std::mutex> lock(query_mutex_);
    query_results_.erase(goal->query_id);
  }

  feedback->finished = true;
  goal_handle->publish_feedback(*feedback);
  goal_handle->succeed(*result);
}

// ----------------- Tell -----------------
rclcpp_action::GoalResponse
ROSInterface::handle_goal_tell(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Tell::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ROSInterface::handle_cancel_tell(
  const std::shared_ptr<GoalHandleTell>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ROSInterface::handle_accepted_tell(
  const std::shared_ptr<GoalHandleTell> goal_handle)
{
  std::thread(&ROSInterface::execute_tell, this, goal_handle).detach();
}

void
ROSInterface::execute_tell(
  const std::shared_ptr<GoalHandleTell> goal_handle)
{
  auto goal = goal_handle->get_goal();
  std::vector<FormulaPtr> formulas;
  for (auto & t : goal->tell.triples) {
    std::vector<TermPtr> terms;
    terms.push_back(IRIAtom::Tabled(t.subject));
    std::string obj = t.object;
    if (obj.front()!='\'' || obj.back()!='\'') {
      obj = "'" + obj + "'";
    }
    terms.push_back(QueryParser::parseConstant(obj));
    formulas.push_back(std::make_shared<Predicate>(t.predicate, terms));
  }
  FormulaPtr phi =
    std::make_shared<Conjunction>(formulas);
  FormulaPtr mPhi = InterfaceUtils::applyModality(
    translateModalityFrameMessage(goal->tell.frame), phi);

  bool ok = InterfaceUtils::assertStatements(kb_, {mPhi});

  auto feedback = std::make_shared<Tell::Feedback>();
  auto result   = std::make_shared<Tell::Result>();
  result->status   = ok
    ? Tell::Result::TRUE
    : Tell::Result::TELL_FAILED;
  feedback->finished = true;
  goal_handle->publish_feedback(*feedback);
  goal_handle->succeed(*result);
}

// ----------------- Services -----------------
void
ROSInterface::handle_ask_incremental_finish(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<srv::AskIncrementalFinish::Request> req,
  std::shared_ptr<srv::AskIncrementalFinish::Response> res)
{
  std::lock_guard<std::mutex> lock(query_mutex_);
  res->success = (query_results_.erase(req->query_id) > 0);
}

void
ROSInterface::handle_export_triples(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<srv::ExportTriples::Request> req,
  std::shared_ptr<srv::ExportTriples::Response> res)
{
  if (req->format == "rdfxml") {
    kb_->exportTo(req->path, semweb::RDF_XML);
    res->success = true;
  } else if (req->format == "turtle") {
    kb_->exportTo(req->path, semweb::TURTLE);
    res->success = true;
  } else {
    RCLCPP_ERROR(get_logger(), "Unsupported export format: %s", req->format.c_str());
    res->success = false;
  }
}
