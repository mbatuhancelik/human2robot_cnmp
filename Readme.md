# Human-to-Robot Skill Transfer with Blending CNMPs

This repository contains the code for transferring a reaching skill between a human demonstrator and the **Torobo** manipulator, using an extension of the **blending Conditional Neural Movement Primitives (CNMP)** framework proposed by Aktaş et al. (2023).

The original blending-CNMP framework learns a shared latent space between robots with different morphologies from clean, robot-to-robot sensorimotor data. This project extends it to a human-in-the-loop setting: human arm-reaching movements are captured with an Intel RealSense camera + MediaPipe, filtered and normalized to counteract the low-frequency drift inherent to vision-based tracking, and used alongside Torobo joint-space trajectories to train a shared latent space. Once trained, a demonstration from either side (human or robot) can generate the corresponding trajectory for the other — including bridging the gap between human Cartesian hand coordinates and Torobo's joint-space control.

A full write-up (background, method, results) is available on my website: **[Project page →](mbatuhancelik.github.io/portfolio/correspondence_learning)**

**Demo video (human → robot transfer):** https://www.youtube.com/watch?v=71mbbTE65yU

> **Note:** This is an independent project extending the method proposed in Aktaş et al. (2023) — I am **not an author** on that paper. See [Background](#background) below.

---

## Background

The blending-CNMP framework encodes each agent's demonstration (a few sampled observation points from a trajectory) into a latent vector using a per-agent encoder, blends the latent vectors of all agents into one shared representation via a randomly-weighted convex combination, and decodes that shared representation back into each agent's own trajectory using per-agent decoders. At inference time, setting the blending weight of one agent to 1 (and the rest to 0) lets a single agent's demonstration drive the shared representation, which is then decoded through another agent's decoder — achieving task-level skill transfer without paired trajectories or a hyperparameter-tuned alignment loss.

This project extends that framework to a human demonstrator, which introduces two challenges the original paper's evaluation didn't cover:
- **Noisy, vision-based input.** Human trajectories tracked with a RealSense camera + MediaPipe carry low-frequency drift and spatial wobble, which the blending-CNMP encoder turned out to be quite sensitive to. A filtering/normalization pipeline (`data.py`) conditions the raw tracked trajectories before they reach the encoder.
- **Cartesian-to-joint-space mapping.** The human side is tracked in Cartesian wrist coordinates; the Torobo side trains and decodes directly in joint space, so the shared latent representation has to implicitly bridge that gap as well.

See the [project page](#) for the full method description and results (max end-effector error ≈ 3 cm across validation trials, bidirectional transfer).

## Repository Structure

```
torobo_robot/          ROS interface for Torobo manipulators (Tokyo Robotics)
data.py                 Human trajectory filtering/normalization pipeline
data.ipynb              Notebook version / exploration of the data pipeline
dataset_utils.py         Dataset loading and batching utilities
demonstration.py        Demonstration collection (human + robot trajectories)
environment.py          Environment/robot-camera interface setup
models.py                Blending-CNMP model (encoders/decoders)
torobo.py                Torobo-specific control interface
train.ipynb              Training notebook
interactive_test.py     Interactive querying/testing of a trained model
try.py                   Ad-hoc experimentation script
utils.py                 Shared helper functions
```

## Typical Workflow

1. **Collect demonstrations** — `demonstration.py` and `environment.py` record human trajectories (via RealSense + MediaPipe) and Torobo trajectories (via `torobo_robot`'s ROS interface).
2. **Preprocess** — `data.py` / `dataset_utils.py` filter and normalize the raw human trajectories, and prepare both agents' data for training.
3. **Train** — `train.ipynb` trains the blending-CNMP model (`models.py`) on the paired human/Torobo dataset.
4. **Query / test** — `interactive_test.py` (or `try.py`) loads a trained model and generates trajectories for one agent given observations from the other.

## Requirements

This repo does not currently ship a `requirements.txt`; you'll need to install these yourself (versions weren't pinned during development):

- Python 3.x
- PyTorch
- NumPy
- MediaPipe
- OpenCV (`opencv-python`)
- `pyrealsense2` (Intel RealSense SDK Python bindings)
- ROS (for the `torobo_robot` interface — required only if you're driving a physical Torobo)
- Jupyter (for the `.ipynb` notebooks)

**Hardware requirement:** an Intel RealSense camera is required to capture human demonstrations; running against the real robot additionally requires a Torobo manipulator with ROS set up via `torobo_robot`.

## Citation

This code implements an extension of the method proposed in:

```bibtex
@article{aktas2023correspondence,
  title={Correspondence learning between morphologically different robots via task demonstrations},
  author={Aktas, Hakan and Nagai, Yukie and Asada, Minoru and Oztop, Erhan and Ugur, Emre},
  journal={arXiv preprint arXiv:2310.13458},
  year={2023}
}
```

which itself builds on:

```bibtex
@inproceedings{seker2019conditional,
  title={Conditional neural movement primitives},
  author={Seker, M. Yunus and Imre, Mert and Piater, Justus H. and Ugur, Emre},
  booktitle={Robotics: Science and Systems (RSS)},
  volume={10},
  year={2019}
}
```

## Acknowledgments

This project was completed during a summer internship at the Symbiotic Intelligent Systems Research Center (SISREC), Osaka University under supervision of Prof. Erhan Öztop, with the experiment idea proposed by Prof. Minoru Asada.
