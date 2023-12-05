# GoMa (Geometric Matching)

## About

goma is a geometric library that implements various single-view (e.g.
vanishing point estimation) and two-view geometry (e.g. homography) functions.

The code is heavily based on [COLMAP](https://github.com/colmap/colmap): it
adopts a similiar file structure and whenever possible, the code links to the
relevant colmap `structs`, `classes` and `functions` (e.g.
`colmap::CameraModel`) or edits the existing ones (e.g. `Image`) to accomodate
the geometric functionalities in `goma` while minimizing the code complexity
by removing code that is not directly necessary.

## Installation

The code has been tested under Ubuntu 18.04 with cmake 3.10, gcc-7.5.0, CUDA-10.0, and COLMAP 3.7.

### Requirements

#### Colmap

The current version of the code is compatible with COLMAP 3.7.
To install it, please follow the instructions in the [colmap website](https://colmap.github.io/install.html)

### Build
```
cmake .
make
```

## Usage

Run `./bin/goma` to list all available commands.

### Examples

See examples in `scripts/` that can be used on the [provided data
sample](https://drive.google.com/file/d/1abuyQkss29X64DzQvZhT_A_zOvuf7Xzz/view?usp=drive_link).

Download the data sample and place the content under `data/`. 

### Commands

- `line_segment_detector`: LSD line segment detection
- `vanishing_point_detector`: Estimates vanishing points in the image. Assumes
  that there is only one vertical vanishing points but there is no Manhattan or
  Atlanta assumption.
- `planewise_homography_estimator`: Given a set of matching 3D planes, their
  vanishing directions and a set of point matches between the planes, estimates
  a homography between plane pairs where the homography is constrained by the
  planes' vanishing directions.

# Citation

If you use this project for your research, please cite:
- the relevant [COLMAP references](https://colmap.github.io/#about)
- the [following publication](https://arxiv.org/abs/2202.04445):

```
@inproceedings{benbihi2022object,
  title={Object-Guided Day-Night Visual localization in Urban Scenes},
  author={Benbihi, Assia and Pradalier, C{\'e}dric and Chum, Ond{\v{r}}ej},
  booktitle={2022 26th International Conference on Pattern Recognition (ICPR)},
  pages={3786--3793},
  year={2022},
  organization={IEEE}
}
```

Developed by [Assia Benbihi](https://abenbihi.github.io/)
