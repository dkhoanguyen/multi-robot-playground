
class SpawnLocalPlanner(object):
    def __init__(self, robot_name: str,
                 robot_namespace: str,
                 robot_idx: int):
        self._robot_name = robot_name
        self._robot_namespace = robot_namespace
        self._robot_idx = robot_idx

        self._node = None
        self._client = None