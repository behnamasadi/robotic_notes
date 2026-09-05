# Large files, datasets and reference books

This repository is meant to stay small enough to clone quickly. Large binaries are handled in
one of three ways depending on what they are.

## 1. Reference books — not in this repository

The textbooks this repo refers to are **not redistributed here**. They are third-party works,
they are freely available from their authors or publishers, and carrying ~130 MB of PDFs in git
history made every clone slow for everyone who forked the repo.

Download them yourself into `pdf/` (which is git-ignored):

| Book | Where to get it |
|---|---|
| Gao, Zhang et al. — *14 Lectures on Visual SLAM* (`slambook-en.pdf`) | <https://github.com/gaoxiang12/slambook-en> (author's own repo, English edition) |
| *SLAM Handbook: From Localization and Mapping to Spatial Intelligence* | <https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release> |
| Barfoot — *State Estimation for Robotics* | <http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf> (author's page, free PDF) |
| Lynch & Park — *Modern Robotics: Mechanics, Planning, and Control* | <https://hades.mech.northwestern.edu/index.php/Modern_Robotics> (free PDF + exercises) |
| Sola — *Quaternion kinematics for the error-state Kalman filter* | <https://arxiv.org/abs/1711.02508> |
| Sola et al. — *A micro Lie theory for state estimation in robotics* | <https://arxiv.org/abs/1812.01537> |
| Durrant-Whyte & Bailey — *SLAM Part I: The Essential Algorithms* | Search the title; IEEE R&A Magazine 2006 |

Quick fetch for the two most used:

```bash
mkdir -p pdf
curl -L -o pdf/barfoot_ser.pdf http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf
curl -L -o pdf/quaternion_kinematics_esekf.pdf https://arxiv.org/pdf/1711.02508
```

## 2. Repository assets — Git LFS

Binary assets that genuinely belong to this repo (simulation meshes and textures, figures,
animated GIFs, notebooks with embedded output, benchmark ground-truth trajectories) are stored
with **Git LFS**. They total well under 100 MB, which fits comfortably in GitHub's free LFS
allowance.

**You need `git-lfs` installed before cloning**, otherwise you get small text pointer files
instead of the real content:

```bash
# Ubuntu / Debian
sudo apt install git-lfs
git lfs install          # once per machine

git clone https://github.com/behnamasadi/robotic_notes.git
```

Already cloned without LFS, or seeing files whose contents look like
`version https://git-lfs.github.com/spec/v1`? Fetch the real data:

```bash
git lfs install
git lfs pull
```

Check what is tracked, and what is actually downloaded:

```bash
git lfs track          # patterns being tracked (from .gitattributes)
git lfs ls-files       # files under LFS in the current checkout
```

To add a new large asset:

```bash
git lfs track "*.dae"        # writes the pattern into .gitattributes
git add .gitattributes path/to/asset.dae
git commit -m "add asset"
```

## 3. Datasets — never in git

Benchmark datasets (Newer College, EuRoC, KITTI, …) are **never** committed, not even via LFS.
They are tens of gigabytes, they are published and versioned by their own maintainers, and a
copy in git history can never be removed cleanly.

`data/slam/` and `lio_benchmark/data/` are git-ignored. Put datasets there, or symlink them in
from wherever you keep large storage:

```bash
mkdir -p "data/slam"
ln -s /path/to/your/storage/newer_college "data/slam/collection 1 - newer college"
```

Newer College Dataset: <https://ori-drs.github.io/newer-college-dataset/>
EuRoC MAV: <https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets>

## Rule of thumb

| Kind of file | Where it goes |
|---|---|
| Source, notes, configs, small figures (< 1 MB) | committed normally |
| Repo's own binaries: meshes, textures, GIFs, GT trajectories | **Git LFS** |
| Third-party books and papers | not committed — see the table above |
| Datasets of any size | not committed — git-ignored, symlinked from external storage |
