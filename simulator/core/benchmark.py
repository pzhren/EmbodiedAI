import os
from collections import defaultdict
from typing import Dict, Optional

from tqdm import tqdm

from simulator.core.dataset import DatasetLoader
from simulator.core.agent import Agent
from simulator.core.env import Env


class Benchmark:
    r"""Benchmark for evaluating agents in environments."""

    def __init__(
        self, root_dir: Optional[str] = "resource/datasets/all_task",
        scene_paths: Optional[str] = "resource/datasets/all_task/scene_datasets",
        robot_paths: Optional[str] = "tests/stretch/model/stretch.usd", 
        eval_remote: bool = False
    ) -> None:
        r"""..

        :param config_paths: file to be used for creating the environment
        :param eval_remote: boolean indicating whether evaluation should be run remotely or locally
        """
        self.dataset_loader = DatasetLoader(
            root_dir=dataset_paths,
            scene_path="scene_datasets",
            robot_path="robot_datasets",
            shuffle=False,
            task_type="single",
            device_id=0,
            headless=True,
        )
        config_env = get_config(config_paths)
        self._eval_remote = eval_remote
        self.eposides_num = len(self.dataset_loader)

    def remote_evaluate(
        self, agent: "Agent", num_episodes: Optional[int] = None
    ):
        # The modules imported below are specific to habitat-challenge remote evaluation.
        # These modules are not part of the habitat-lab repository.
        import pickle
        import time

        import evalai_environment_habitat  # noqa: F401
        import evaluation_pb2
        import evaluation_pb2_grpc
        import grpc

        time.sleep(60)

        def pack_for_grpc(entity):
            return pickle.dumps(entity)

        def unpack_for_grpc(entity):
            return pickle.loads(entity)

        def remote_ep_over(stub):
            res_env = unpack_for_grpc(
                stub.episode_over(evaluation_pb2.Package()).SerializedEntity
            )
            return res_env["episode_over"]

        env_address_port = os.environ.get("EVALENV_ADDPORT", "localhost:8085")
        channel = grpc.insecure_channel(env_address_port)
        stub = evaluation_pb2_grpc.EnvironmentStub(channel)

        base_num_episodes = unpack_for_grpc(
            stub.num_episodes(evaluation_pb2.Package()).SerializedEntity
        )
        num_episodes = base_num_episodes["num_episodes"]

        agg_metrics: Dict = defaultdict(float)

        count_episodes = 0

        while count_episodes < num_episodes:
            agent.reset()
            
            res_env = unpack_for_grpc(
                stub.reset(evaluation_pb2.Package()).SerializedEntity
            )


            while not remote_ep_over(stub):
                obs = res_env["observations"]
                action = agent.act(obs)

                res_env = unpack_for_grpc(
                    stub.act_on_environment(
                        evaluation_pb2.Package(
                            SerializedEntity=pack_for_grpc(action)
                        )
                    ).SerializedEntity
                )

            metrics = res_env["info"]

            for m, v in metrics.items():
                agg_metrics[m] += v
            count_episodes += 1

        avg_metrics = {k: v / count_episodes for k, v in agg_metrics.items()}

        stub.evalai_update_submission(evaluation_pb2.Package())

        return avg_metrics

    def local_evaluate(
        self, agent: "Agent", num_episodes: Optional[int] = None
    ) -> Dict[str, float]:
        if num_episodes is None:
            num_episodes = len(self.episodes)
        else:
            assert num_episodes <= len(self.episodes), (
                "num_episodes({}) is larger than number of episodes "
                "in environment ({})".format(
                    num_episodes, len(self.episodes)
                )
            )

        assert num_episodes > 0, "num_episodes should be greater than 0"

        agg_metrics: Dict = defaultdict(float)

        count_episodes = 0

        pbar = tqdm(total=num_episodes)
        while count_episodes < num_episodes:
            self._env = Env(self.dataset_loader[count_episodes])
            observations, info, _, done = self._env.reset()
            agent.reset()

            while not self._env.is_over():
                action = agent.act(observations)
                observations, info,_, done = self._env.step(action)

            metrics = info
            for m, v in metrics.items():
                if isinstance(v, dict):
                    for sub_m, sub_v in v.items():
                        agg_metrics[m + "/" + str(sub_m)] += sub_v
                else:
                    agg_metrics[m] += v
            count_episodes += 1
            pbar.update(1)

        avg_metrics = {k: v / count_episodes for k, v in agg_metrics.items()}

        return avg_metrics

    def evaluate(
        self, agent: "Agent", num_episodes: Optional[int] = None
    ) -> Dict[str, float]:
        r"""..

        :param agent: agent to be evaluated in environment.
        :param num_episodes: count of number of episodes for which the
            evaluation should be run.
        :return: dict containing metrics tracked by environment.
        """

        if self._eval_remote is True:
            return self.remote_evaluate(agent, num_episodes)
        else:
            return self.local_evaluate(agent, num_episodes)