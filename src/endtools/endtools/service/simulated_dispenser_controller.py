import math
import os
import xacro
from threading import RLock
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode
from rclpy.time import Time

# NOTE: TransformException is re-exported by tf2_ros from the compiled tf2_py
# C-extension via a wildcard import, so Pylance/Pyright cannot resolve it
# statically even though it exists at runtime.
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException  # type: ignore[attr-defined]

from endtools.exception.world_manager_exception import WorldManagerError
from endtools.interface.dispenser_controller import DispenserController
from endtools.model.dispensing_metrics_dto import DispensingMetricsDTO
from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO
from endtools.service.gazebo_world_manager import GazeboWorldManager


class SimulatedDispenserController(DispenserController):
    """Simulated controller for the volumetric dispensing tool."""
    # Travel (mm) above which a dispensed blob is modelled as a stripe rather
    # than a dot. Kept above zero so the sub-millimetre residual of a nominally
    # stationary dwell (planner goal tolerance, controller settling) does not
    # turn a dot into a degenerate capsule.
    _STRIPE_MIN_LENGTH_MM: float = 1.0

    # How far back the TF buffer keeps transforms. Must cover the longest
    # expected dispensing operation so the TCP pose at the start of dispensing
    # can still be looked up once dispensing stops.
    _TF_CACHE_PERIOD_S: float = 60.0

    # How long a single TF lookup may block waiting for the transform.
    _TF_TIMEOUT_S: float = 3.0

    def __init__(self, node: LifecycleNode):
        self._config: DispensingToolConfigDTO | None = None
        self._node = node

        self._is_dispensing = False
        self._lock = RLock()  # Thread-safe lock for concurrent access
        # Node-clock instants, used only to measure how long the valve was open.
        self._dispensing_start_time: Time | None = None
        # Instant the TCP reached the pose the element starts at. Supplied by the
        # caller and deliberately kept separate from the valve-open instant: the
        # two differ by the caller's round-trip latency, and for a dwell ("dot")
        # they differ by the whole dwell duration.
        self._dispensing_start_pose_stamp: Time | None = None

        # TODO: these are hardcoded to match the 'mock_product' entity in the
        # Gazebo world; in final implementation these should come
        # from a topic (published either by product localizer or router in SIM mode)
        self._product_model_name: str = "mock_product"
        self._product_link: str = "product_origin"
        self._product_frame_id: str = "product_origin"

        self._gazebo_world_manager: GazeboWorldManager | None = None
        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None

    @property
    def is_dispensing(self) -> bool:
        with self._lock:
            return self._is_dispensing

    @is_dispensing.setter
    def is_dispensing(self, value: bool):
        with self._lock:
            self._is_dispensing = value

    @staticmethod
    def _generate_model_name(prefix: str) -> str:
        """Generate a unique Gazebo model name for a dispensed material blob.

        The name must be unique across the world and must differ from the product
        model name, since the DetachableJoint plugin references the product as its
        ``child_model`` and the plugin topics are namespaced by this name.
        """
        return f'{prefix}_{uuid4().hex}'

    def _get_stripe_sdf(
        self,
        model_name: str,
        product_model_name: str,
        product_link: str,
        radius_mm: float,
        length_mm: float
    ) -> str:
        model_path = os.path.join(
            get_package_share_directory("endtools"),
            "sdf",
            "stripe.sdf.xacro"
        )
        document = xacro.process_file(
            model_path,
            mappings={
                'model_name': model_name,
                'product_model_name': product_model_name,
                'product_link': product_link,
                # SDF geometry is expressed in metres, inputs are millimetres
                'radius_m': str(radius_mm / 1000.0),
                'length_m': str(length_mm / 1000.0),
            },
        )
        return document.toxml()  # type: ignore

    def _get_dot_sdf(
        self,
        model_name: str,
        product_model_name: str,
        product_link: str,
        radius_mm: float
    ) -> str:
        model_path = os.path.join(
            get_package_share_directory("endtools"),
            "sdf",
            "dot.sdf.xacro"
        )
        document = xacro.process_file(
            model_path,
            mappings={
                'model_name': model_name,
                'product_model_name': product_model_name,
                'product_link': product_link,
                # SDF geometry is expressed in metres, inputs are millimetres
                'radius_m': str(radius_mm / 1000),
            },
        )
        return document.toxml()  # type: ignore

    @staticmethod
    def _calculate_capsule_radius(length_mm: float, volume_mm3: float) -> float:
        """Solve for the radius of a capsule of a given length and volume.

        A stripe is modelled as a cylinder of length ``length_mm`` capped by a
        sphere at each end, all sharing the same radius. Because a cap sphere is
        centred on the cylinder end face, exactly one hemisphere of it lies
        inside the cylinder, so the union volume is::

            V = pi * r^2 * L + (4 / 3) * pi * r^3

        That cubic is strictly increasing in ``r`` for ``r > 0``, so it is
        inverted by bisection. A sphere holding the full volume bounds the
        solution from above, since adding the cylinder term can only increase
        the volume at a given radius.

        Args:
            length_mm: Cylinder length (distance between the cap centres), in mm.
            volume_mm3: Total capsule volume, in cubic millimetres.

        Returns:
            The capsule radius in millimetres.

        Raises:
            ValueError: If the volume is not positive or the length is negative.
        """
        if volume_mm3 <= 0.0:
            raise ValueError(f'volume_mm3 must be > 0, got {volume_mm3}')
        if length_mm < 0.0:
            raise ValueError(f'length_mm must be >= 0, got {length_mm}')

        def volume_at(radius: float) -> float:
            return (
                math.pi * radius ** 2 * length_mm
                + (4.0 / 3.0) * math.pi * radius ** 3
            )

        low = 0.0
        high = (3.0 * volume_mm3 / (4.0 * math.pi)) ** (1.0 / 3.0)
        for _ in range(100):
            middle = 0.5 * (low + high)
            if volume_at(middle) < volume_mm3:
                low = middle
            else:
                high = middle

        return 0.5 * (low + high)

    def _generate_sdf_content(
        self,
        product_model_name: str,
        product_link: str,
        length_mm: float,
        volume_mm3: float,
    ) -> tuple[str, str]:
        """
        Generate SDF content to spawn a simulated dispensed putty material model. The
        stripes are made up of two speheres and a connecting cylinder, all having the
        same radius. The dots are simple spheres.

        The length is used to determine if its a stripe or a dot. The length, together
        with volume, is also used to calculate the radius of the spheres and cylinder.

        Args:
            product_model_name: Name of the spawned product model.
            product_link: Name of the product link in the product model.
            length_mm: Length of the dispensed material in millimeters.
            volume_mm3: Volume of the dispensed material in cubic millimeters.

        Returns:
            Tuple of the unique model name and the SDF content as a string. The model
            name must be reused when spawning so the entity name matches the
            DetachableJoint topic namespace baked into the SDF.
        """
        if length_mm > self._STRIPE_MIN_LENGTH_MM:
            model_name = self._generate_model_name('stripe')
            radius_mm = self._calculate_capsule_radius(length_mm, volume_mm3)
            return model_name, self._get_stripe_sdf(
                model_name,
                product_model_name,
                product_link,
                radius_mm,
                length_mm
            )
        else:  # DOT
            model_name = self._generate_model_name('dot')
            radius_mm = (3 * volume_mm3 / (4 * math.pi)) ** (1 / 3)
            return model_name, self._get_dot_sdf(
                model_name,
                product_model_name,
                product_link,
                radius_mm
            )

    def _lookup_tcp_position(self, stamp: Time) -> tuple[float, float, float]:
        """Look up the calibrated TCP position relative to the product frame.

        Falls back to the latest available transform when the buffer cannot
        resolve the requested instant (for example when the dispensing operation
        outlived the TF cache period).

        Args:
            stamp: Instant to resolve the TCP pose at.

        Returns:
            The TCP position ``(x, y, z)`` in metres, expressed in the product frame.

        Raises:
            TransformException: If neither the requested instant nor the latest
                transform could be resolved.
        """
        if self._config is None or self._tf_buffer is None:
            raise RuntimeError('Controller is not set up; cannot look up TCP pose.')

        tcp_frame_id = self._config.calibrated_tcp_frame_id
        timeout = Duration(seconds=self._TF_TIMEOUT_S)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._product_frame_id, tcp_frame_id, stamp, timeout=timeout
            )
        except TransformException as exc:
            self._node.get_logger().warning(
                f"Failed to look up '{tcp_frame_id}' in '{self._product_frame_id}' at the "
                f'requested time ({exc}); falling back to the latest transform.'
            )
            # Time() is the zero stamp, which tf2 resolves to "latest available".
            transform = self._tf_buffer.lookup_transform(
                self._product_frame_id, tcp_frame_id, Time(), timeout=timeout
            )

        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    def _spawn_dispensed_material(
        self,
        volume_cc: float,
        start_time: Time,
        end_time: Time,
    ) -> str | None:
        """Spawn a model of the dispensed material in the Gazebo world.

        The blob spans the travel of the calibrated TCP between the start and
        the end of the dispensing operation, so both poses are resolved against
        the product frame and the straight-line distance between them drives the
        stripe-versus-dot decision. Failures are logged and swallowed: a missing
        blob must not mask the dispensing metrics.

        Args:
            volume_cc: Dispensed volume in cubic centimetres.
            start_time: Instant the TCP reached the pose the element starts at.
            end_time: Instant the TCP reached the pose the element ends at.

        Returns:
            The spawned model name, or ``None`` if the model could not be spawned.
        """
        if self._gazebo_world_manager is None:
            self._node.get_logger().error(
                'Controller is not set up with a world manager; skipping material spawn.'
            )
            return None

        try:
            start_position = self._lookup_tcp_position(start_time)
            end_position = self._lookup_tcp_position(end_time)
        except (TransformException, RuntimeError) as exc:
            self._node.get_logger().error(
                f'Failed to resolve the dispensing path from TF: {exc}'
            )
            return None

        delta_x = end_position[0] - start_position[0]
        delta_y = end_position[1] - start_position[1]
        delta_z = end_position[2] - start_position[2]
        length_mm = math.sqrt(
            delta_x ** 2 + delta_y ** 2 + delta_z ** 2
        ) * 1000.0
        volume_mm3 = volume_cc * 1000.0

        if volume_mm3 <= 0.0:
            self._node.get_logger().warning(
                f'Dispensed volume is {volume_cc} cc; skipping material spawn.'
            )
            return None

        if length_mm > self._STRIPE_MIN_LENGTH_MM:
            # The stripe SDF extends along its own +X axis, so the model is
            # rotated to point from the start position towards the end position.
            # Roll is irrelevant for a body of revolution and is left at zero.
            length_m = length_mm / 1000.0
            roll = 0.0
            pitch = -math.asin(max(-1.0, min(1.0, delta_z / length_m)))
            yaw = math.atan2(delta_y, delta_x)
        else:
            roll = pitch = yaw = 0.0

        try:
            model_name, sdf_content = self._generate_sdf_content(
                self._product_model_name,
                self._product_link,
                length_mm,
                volume_mm3,
            )
        except Exception as exc:
            self._node.get_logger().error(
                f'Failed to generate the dispensed material SDF: {exc}'
            )
            return None

        pose = (
            start_position[0],
            start_position[1],
            start_position[2],
            roll,
            pitch,
            yaw,
        )
        try:
            # The model name is reused as the entity name so it matches the
            # DetachableJoint topic namespace baked into the SDF.
            return self._gazebo_world_manager.spawn_model(
                sdf_content=sdf_content,
                pose=pose,
                link_name=self._product_frame_id,
                model_id=model_name,
            )
        except WorldManagerError as exc:
            self._node.get_logger().error(
                f'Failed to spawn dispensed material {model_name}: {exc}'
            )
            return None

    def start_dispensing(self, pose_stamp: Time | None = None):
        """Start the hardware dispensing operation.

        :param pose_stamp: Instant the TCP reached the pose the dispensed element
            starts at. ``None`` falls back to the node clock, which is only
            correct when the caller is not separated from the motion by any
            round-trip latency.
        """
        with self._lock:
            if self.is_dispensing:
                raise RuntimeError("Dispensing is already in progress.")
            self.is_dispensing = True

        now = self._node.get_clock().now()
        self._dispensing_start_time = now
        self._dispensing_start_pose_stamp = pose_stamp if pose_stamp is not None else now

    def stop_dispensing(self, pose_stamp: Time | None = None) -> DispensingMetricsDTO:
        """Stop the hardware dispensing operation and return metrics.

        :param pose_stamp: Instant the TCP reached the pose the dispensed element
            ends at. ``None`` falls back to the node clock.
        """
        with self._lock:
            if not self.is_dispensing:
                raise RuntimeError("Dispensing is not in progress.")
            self.is_dispensing = False

        if self._config is None:
            raise RuntimeError("Controller is not set up with a configuration.")
        if self._dispensing_start_time is None or self._dispensing_start_pose_stamp is None:
            raise RuntimeError("Dispensing start time is unknown.")

        dispensing_end_time = self._node.get_clock().now()
        end_pose_stamp = pose_stamp if pose_stamp is not None else dispensing_end_time
        # The volume comes from how long the valve was actually open; the geometry
        # comes from where the TCP was at the motion boundaries. For a dwell both
        # pose stamps are the same instant, which yields a dot of the dwell volume.
        dispensing_time = dispensing_end_time - self._dispensing_start_time
        dispensed_volume = self._config.flow_rate * dispensing_time.nanoseconds / 1e9  # Convert nanoseconds to seconds

        self._spawn_dispensed_material(
            volume_cc=dispensed_volume,
            start_time=self._dispensing_start_pose_stamp,
            end_time=end_pose_stamp,
        )

        self._dispensing_start_time = None
        self._dispensing_start_pose_stamp = None

        return DispensingMetricsDTO(
            dispensed_volume_cc=dispensed_volume,
            dispensing_duration_s=dispensing_time.nanoseconds / 1e9,
            flowrate_cc=self._config.flow_rate,
        )

    def setup(self, config: DispensingToolConfigDTO) -> None:
        """Set up the hardware controller with the given configuration."""
        self._config = config

        self._tf_buffer = Buffer(
            cache_time=Duration(seconds=self._TF_CACHE_PERIOD_S)
        )
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._gazebo_world_manager = GazeboWorldManager(
            node=self._node,
            world_name=self._config.world_name,
        )

    def teardown(self) -> None:
        """Tear down the hardware controller."""
        self._config = None
        self._dispensing_start_time = None
        self._dispensing_start_pose_stamp = None

        if self._tf_listener is not None:
            self._tf_listener.unregister()
        self._tf_listener = None
        self._tf_buffer = None

        self._gazebo_world_manager = None
