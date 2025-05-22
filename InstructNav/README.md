### InstructNav for InfiniteWorld

#### Env

- Isaac Sim
- InfiniteWorld
- cuda 11.8

```bash
pip install -r requirements.txt
python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
```

#### Checkpoint

- [GLEE_SwinL_Scaleup10m.pth](https://huggingface.co/spaces/Junfeng5/GLEE_demo/blob/main/GLEE_SwinL_Scaleup10m.pth)

- [clip-vit-base-patch32](https://huggingface.co/openai/clip-vit-base-patch32)

#### Dataset

- HSSD scene dataset and task dataset for InfiniteWorld
- Generated scene dataset and task dataset for InfiniteWorld

#### Config

- `./llm_utils/gpt_request.py`: `API_KEY`, your OpenAI API key
- `./constants.py`: `GLEE_CHECKPOINT_PATH` and `CLIP_CHECKPOINT_PATH`, checkpoint path for `GLEE_SwinL_Scaleup10m.pth` and `clip-vit-base-patch32`
- `./eval.py`: `root_dir`, `scene_path` and `robot_path`, path for task dataset, scene dataset and robot dataset

#### Run

```bash
python eval.py
```

