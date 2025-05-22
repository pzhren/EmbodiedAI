# Imitation Learning Project

This project is for building imitation learning datasets and training models.

## Dependencies

It is recommended to use Python 3.8+ and install the following dependencies:

```bash
pip install torch torchvision torchaudio
pip install transformers tqdm pillow numpy
```

Install any additional dependencies as prompted by error messages.

---

## Dataset Building

### 1. Build General Dataset

Use `build_dataset.py` to build the dataset:

```bash
python dataset/build_dataset.py --data_root <raw_data_directory> --output_dir <output_directory>
```

- `--data_root`: Root directory containing the original demo folders
- `--output_dir`: Directory to save the generated `train.json` and `val.json`

### 2. Build HSSD Dataset (Single Underscore Demos Only)

Use `build_hssd_dataset.py` to build a dataset containing only demos with a single underscore:

```bash
python dataset/build_hssd_dataset.py --data_root <raw_data_directory> --output_dir <output_directory>
```

Parameters are the same as above.

---

## Model Training

Use `train.py` to train the model:

```bash
python train.py --data_root <dataset_directory> --save_dir <checkpoint_directory>
```

- `--data_root`: Directory containing `train.json` and `val.json` (i.e., the output_dir from the previous step)
- `--save_dir`: Directory to save model checkpoints (optional, default: `no_gen_scene_checkpoints`)

To customize batch size and other hyperparameters, edit `config/model_config.py`.

---

## Additional Notes

- Log files are saved in the `logs/` directory.
- Training logs for wandb are saved locally in the `wandb/` directory.
- You can resume training from saved checkpoints.

For any questions, please contact the project maintainer.
