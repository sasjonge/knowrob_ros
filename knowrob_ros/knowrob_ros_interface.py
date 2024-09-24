import rclpy
from rclpy.node import Node
from knowrob import *
from rclpy.action import ActionServer
from knowrob_ros_interfaces.action import AskAll, AskOne, Tell

class KnowRobROSInterface(Node):
    def __init__(self):
        super().__init__('knowrob_ros_interface')
        
        # Initialize Knowledge Base
        self.kb = KnowledgeBase("path/to/config.json")
        
        # Define action servers
        self.ask_all_action_server = ActionServer(
            self, AskAll, 'knowrob/askall', self.execute_ask_all_callback
        )
        self.ask_one_action_server = ActionServer(
            self, AskOne, 'knowrob/askone', self.execute_ask_one_callback
        )
        self.tell_action_server = ActionServer(
            self, Tell, 'knowrob/tell', self.execute_tell_callback
        )

    def execute_ask_all_callback(self, goal_handle):
        self.get_logger().info('Executing AskAll query')
        query_string = goal_handle.request.query
        modalities = {}  # Define modalities if needed

        result = self.run_query(query_string, modalities)
        goal_handle.succeed()
        return result

    def execute_ask_one_callback(self, goal_handle):
        self.get_logger().info('Executing AskOne query')
        query_string = goal_handle.request.query
        modalities = {}  # Define modalities if needed

        result = self.run_query(query_string, modalities)
        goal_handle.succeed()
        return result

    def execute_tell_callback(self, goal_handle):
        self.get_logger().info('Executing Tell action')
        query_string = goal_handle.request.fact
        modalities = {}  # Define modalities if needed

        result = self.run_query(query_string, modalities)
        goal_handle.succeed()
        return result

    def run_query(self, query_string, modalities=None):
        if modalities is None:
            modalities = {
                "epistemicOperator": EpistemicOperator.KNOWLEDGE,
                "confidence": 0.0,
                "temporalOperator": TemporalOperator.CURRENTLY,
            }
        # Create a formula for the query
        phi = QueryParser.parse(query_string)
        mPhi = InterfaceUtils.applyModality(modalities, phi)
        
        # Submit query to KnowRob knowledge base
        result_stream = self.kb.submitQuery(mPhi, QueryContext(QueryFlag.QUERY_FLAG_ALL_SOLUTIONS))
        results = result_stream.createQueue()

        return results

def main(args=None):
    rclpy.init(args=args)
    knowrob_ros_interface = KnowRobROSInterface()
    rclpy.spin(knowrob_ros_interface)
    knowrob_ros_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
