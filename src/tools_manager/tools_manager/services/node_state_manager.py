from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.parameter import ParameterValue

class NodeStateManager():

    def __init__(self, node: LifecycleNode, node_name: str) -> None:
        self._node = node
        self._node_name = node_name

    def configure_node(self) -> TransitionCallbackReturn:
        """
        Configures the given node by calling its on_configure method.

        :param node: The ROS 2 lifecycle node to configure.
        :type node: LifecycleNode
        """
        self._node.get_logger().info(f'Configuring node: {self._node_name}')
        raise NotImplementedError()

    def activate_node(self) -> None:
        """
        Activates the given node by calling its on_activate method.

        :param node: The ROS 2 lifecycle node to activate.
        :type node: LifecycleNode
        """
        self._node.get_logger().info(f'Activating node: {self._node_name}')
        raise NotImplementedError()

    def deactivate_node(self) -> TransitionCallbackReturn:
        """
        Deactivates the given node by calling its on_deactivate method.

        :param node: The ROS 2 lifecycle node to deactivate.
        :type node: LifecycleNode
        """
        self._node.get_logger().info(f'Deactivating node: {self._node_name}')
        raise NotImplementedError()

    def unconfigure_node(self) -> TransitionCallbackReturn:
        """
        Unconfigures the given node by calling its on_cleanup method.

        :param node: The ROS 2 lifecycle node to unconfigure.
        :type node: LifecycleNode
        """
        self._node.get_logger().info(f'Unconfiguring node: {self._node_name}')
        raise NotImplementedError()

    def get_node_state(self) -> TransitionCallbackReturn:
        """
        Returns the current state of the given node.

        :return: The current state of the node as a string.
        :rtype: str
        """
        raise NotImplementedError()

    def set_parameter(self, parameter_name: str, parameter_value: ParameterValue) -> bool:
        """
        Sets a parameter on the given node.

        :param node: The ROS 2 lifecycle node to set the parameter on.
        :type node: LifecycleNode
        :param parameter_name: The name of the parameter to set.
        :type parameter_name: str
        :param parameter_value: The value of the parameter to set.
        """
        self._node.get_logger().info(f'Setting parameter {parameter_name} on node: {self._node_name}')
        raise NotImplementedError()

    def get_parameter(self, parameter_name: str) -> ParameterValue:
        """
        Gets a parameter from the given node.

        :param node: The ROS 2 lifecycle node to get the parameter from.
        :type node: LifecycleNode
        :param parameter_name: The name of the parameter to get.
        :type parameter_name: str
        :return: The value of the parameter.
        :rtype: ParameterValue
        """
        raise NotImplementedError()
